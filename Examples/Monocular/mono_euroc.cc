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

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps);
void LoadImagesCSV(const string &strImagePath, const string &strPathTimes,
                   vector<string> &vstrImages, vector<double> &vTimeStamps);
int main(int argc, char **argv)
{

    // Load all sequences:
    int seq;
    vector<string> vstrImageFilenames;
    vector<double> vTimestampsCam;
    int nImages;

    string datasetPath(argv[3]);
//    string imagePath = datasetPath + "/mav0/cam0/data";
//    string imageFile = datasetPath + "/mav0/cam0/data.csv";
    string imagePath = datasetPath + "/camera/images";
    string imageFile = datasetPath + "/camera/data.csv";

//    LoadImages(imagePath, string(argv[4]), vstrImageFilenames, vTimestampsCam);
    LoadImagesCSV(imagePath, imageFile, vstrImageFilenames, vTimestampsCam);
    nImages = vstrImageFilenames.size();


    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout.precision(17);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR, true);

    #if defined(__APPLE__)
        // Moving main loop to a separate thread so that we could run UI thread on the main thread.
        std::thread runthread([&]() {  // Start in new thread
    #endif
        // Main loop
        cv::Mat im;
        int proccIm = 0;
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

    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
    #endif

            // Pass the image to the SLAM system
            // cout << "tframe = " << tframe << endl;
            SLAM.TrackMonocular(im,tframe); // TODO change to monocular_inertial

    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
    #endif

            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

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
