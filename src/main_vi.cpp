#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <ctime>
#include <sstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<System.h>
#include "ImuTypes.h"

using namespace std;

double et = 1;

//zhouwei add csv file load
void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps)
{
    ifstream f;
    f.open(strPathTimes.c_str());
    if (!f.is_open())
    {
        cout << "Can't open file " << strPathTimes << endl;
    }

    string s0;
    getline(f, s0);
    int sz = strPathTimes.find("csv") ;
    bool isCSV = sz >0 ;
    while (!f.eof())
    {
        string s;
        getline(f, s);
        if (!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            char quart;
            string sRGB;
            ss >> t;
            vTimeStamps.push_back(t/et);
            if (isCSV)
            {
                ss >> quart;
            }
            ss >> sRGB;
            vstrImages.push_back(strImagePath + "/" +sRGB);
        }
    }
}

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro)
{
    ifstream fImu;
    fImu.open(strImuPath.c_str());
    vTimeStamps.reserve(5000);
    vAcc.reserve(5000);
    vGyro.reserve(5000);

    string s;
    getline(fImu,s);

    while(!fImu.eof())
    {
        string s;
        getline(fImu,s);
        if(!s.empty())
        {
            string item;
            size_t pos = 0;
            double data[7];
            int count = 0;
            while ((pos = s.find(',')) != string::npos)
            {
                item = s.substr(0, pos);
                data[count++] = stod(item);
                s.erase(0, pos + 1);
            }
            item = s.substr(0, pos);
            data[6] = stod(item);

            vTimeStamps.push_back(data[0]/et);
            vAcc.push_back(cv::Point3f(data[4],data[5],data[6]));
            vGyro.push_back(cv::Point3f(data[1],data[2],data[3]));
        }
    }
}


int main(int argc, char **argv)
{
    double ttrack_tot = 0;
    cout.precision(12);
    vector<string> vstrImageFilenames;
    vector<double> vTimestampsCam;
    vector<cv::Point3f> vAcc, vGyro;
    vector<double> vTimestampsImu;

    int first_imu = 0;

    string dataset = (string)argv[1];
    string rgbpath = dataset + "/camera/images";
    string imuFile = dataset + "/imu/data.csv";
    string configPath = (string)argv[2];
//    string vocPath = configPath + "/orbvoc_9_5.bin";
    string vocPath = configPath + "/ORBvoc.bin";
    string strFile = dataset + "/camera/data.csv";

    string yamlPath = configPath + "/st.yaml";

    LoadImages(rgbpath, strFile, vstrImageFilenames, vTimestampsCam);
	LoadIMU(imuFile, vTimestampsImu, vAcc, vGyro);

    int nImages = vstrImageFilenames.size();
    int nImu = vTimestampsImu.size();

    if((nImages<=0)||(nImu<=0))
    {
        cerr << "ERROR: Failed to load images or IMU" << endl;
        return 1;
    }

    while(vTimestampsImu[first_imu]<=vTimestampsCam[0])
    {
        first_imu++;
    }
    first_imu--;

    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(vocPath, yamlPath, ORB_SLAM3::System::IMU_MONOCULAR, true);
    int proccIm = 0;
#ifdef __APPLE__
        // Moving main loop to a separate thread so that we could run UI thread on the main thread.
        std::thread runthread([&]() {  // Start in new thread
#endif

    cv::Mat im;
    vector<ORB_SLAM3::IMU::Point> vImuMeas;
    proccIm = 0;
    for (std::size_t ni = 0; ni < nImages; ni++, proccIm++)
    {
        im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);

        double tframe = vTimestampsCam[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: " <<  vstrImageFilenames[ni] << endl;
            return 1;
        }

        vImuMeas.clear();

        if(ni>0)
        {
            while(vTimestampsImu[first_imu]<=vTimestampsCam[ni])
            {

                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(vAcc[first_imu].x,vAcc[first_imu].y,vAcc[first_imu].z,
                                                             vGyro[first_imu].x,vGyro[first_imu].y,vGyro[first_imu].z,
                                                             vTimestampsImu[first_imu]));
                    first_imu++;
            }
        }
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
//        SLAM.TrackMonocular(im, vTimestampsCam[ni]);
        SLAM.TrackMonocular(im,vTimestampsCam[ni],vImuMeas); // TODO change to monocular_inertial

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        ttrack_tot += ttrack;
        vTimesTrack[ni]=ttrack;
        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestampsCam[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestampsCam[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6); // 1e6
    }


#ifdef __APPLE__
        SLAM.StopViewer();
    });

    SLAM.StartViewer();
    runthread.join();
#endif

    // Stop all threads
    SLAM.Shutdown();

    return 0;
}
//zhouwei add csv file load

