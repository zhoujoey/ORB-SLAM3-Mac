#include <iostream>
#include <chrono>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <System.h>

#include <time.h>
#include <string>

#include <memory>

using namespace std;

double et = 1e9;

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

int main(int argc, char **argv)
{

    // Load all sequences:
    vector<string> vstrImageFilenames;
    vector<double> vTimestampsCam;


    string dataset = (string)argv[1];
    string rgbpath = dataset + "/camera/images";
    string imuFile = dataset + "/imu/data.csv";
    string configPath = (string)argv[2];
//    string vocPath = configPath + "/orbvoc_9_5.bin";
    string vocPath = configPath + "/ORBvoc.bin";
    string strFile = dataset + "/camera/data.csv";

    string yamlPath = configPath + "/st.yaml";
    LoadImages(rgbpath, strFile, vstrImageFilenames, vTimestampsCam);


    int nImages = vstrImageFilenames.size();

    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout.precision(17);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(vocPath, yamlPath, ORB_SLAM3::System::MONOCULAR, true);

#ifdef __APPLE__
        // Moving main loop to a separate thread so that we could run UI thread on the main thread.
        std::thread runthread([&]() {  // Start in new thread
#endif

    cv::Mat im;

    int proccIm = 0;

    for (std::size_t ni = 0; ni < nImages; ni++, proccIm++)
    {
        im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestampsCam[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: " <<  vstrImageFilenames[ni] << endl;
            return 1;
        }

        SLAM.TrackMonocular(im, vTimestampsCam[ni]);

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

