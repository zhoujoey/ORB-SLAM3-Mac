/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <ctime>
#include <sstream>

#include<opencv2/core/core.hpp>

#include<System.h>
#include "ImuTypes.h"

using namespace std;

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps);
void LoadImagesCSV(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps);
void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro);

double ttrack_tot = 0;
int main(int argc, char *argv[])
{
//    if(argc < 5)
//    {
//        cerr << endl << "Usage: ./mono_inertial_euroc path_to_vocabulary path_to_settings path_to_sequence_folder_1 path_to_times_file_1 (path_to_image_folder_2 path_to_times_file_2 ... path_to_image_folder_N path_to_times_file_N) " << endl;
//        return 1;
//    }


    vector<string> vstrImageFilenames;
    vector<double> vTimestampsCam;
    vector<cv::Point3f> vAcc, vGyro;
    vector<double> vTimestampsImu;
    int nImages;
    int nImu;
    int first_imu;


    //load paths
    string datasetPath(argv[3]);
    string imagePath = datasetPath + "/mav0/cam0/data";
    string imageFile = datasetPath + "/mav0/cam0/data.csv";
    string imuPath = datasetPath + "/mav0/imu0/data.csv";

    //load images and imus
    LoadImagesCSV(imagePath, imageFile, vstrImageFilenames, vTimestampsCam);
    LoadIMU(imuPath, vTimestampsImu, vAcc, vGyro);


    nImages = vstrImageFilenames.size();
    nImu = vTimestampsImu.size();

    if((nImages<=0)||(nImu<=0))
    {
        cerr << "ERROR: Failed to load images or IMU" << endl;
        return 1;
    }

    // Find first imu to be considered, supposing imu measurements start first

    while(vTimestampsImu[first_imu]<=vTimestampsCam[0])
        first_imu++;
    first_imu--; // first imu measurement to be considered


    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout.precision(17);


    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_MONOCULAR, true);

    int proccIm=0;

#if defined(__APPLE__)
    // Moving main loop to a separate thread so that we could run UI thread on the main thread.
    std::thread runthread([&]() {  // Start in new thread
#endif
    // Main loop
    cv::Mat im;
    vector<ORB_SLAM3::IMU::Point> vImuMeas;
    proccIm = 0;
    for(int ni=0; ni<nImages; ni++, proccIm++)
    {
        // Read image from file
        im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);

        double tframe = vTimestampsCam[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "
                 <<  vstrImageFilenames[ni] << endl;
            return 1;
        }

        // Load imu measurements from previous frame
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


#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,tframe,vImuMeas); // TODO change to monocular_inertial

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

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

#if defined(__APPLE__)
    // stop viewer so that we could exit UI / main thread
    SLAM.StopViewer();
    });

    // OSX requires calling visualization on main thread
    SLAM.StartViewer();
    runthread.join();
#endif


    // Stop all threads
    SLAM.Shutdown();


    return 0;
}

//zhouwei add csv file load
void LoadImagesCSV(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if (s[0] == '#')
            continue;
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            char quart;
            string sRGB;
            ss>>t;
            ss>>quart;
            ss>>sRGB;
            vstrImages.push_back(strImagePath + "/" +sRGB);
            vTimeStamps.push_back(t/1e9);
        }
    }
}
void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t/1e9);

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

    while(!fImu.eof())
    {
        string s;
        getline(fImu,s);
        if (s[0] == '#')
            continue;

        if(!s.empty())
        {
            string item;
            size_t pos = 0;
            double data[7];
            int count = 0;
            while ((pos = s.find(',')) != string::npos) {
                item = s.substr(0, pos);
                data[count++] = stod(item);
                s.erase(0, pos + 1);
            }
            item = s.substr(0, pos);
            data[6] = stod(item);

            vTimeStamps.push_back(data[0]/1e9);
            vAcc.push_back(cv::Point3f(data[4],data[5],data[6]));
            vGyro.push_back(cv::Point3f(data[1],data[2],data[3]));
        }
    }
}
