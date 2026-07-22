/**
* This file is part of ORB-SLAM3
*
* [N6] EuRoC-layout RGB-D-inertial runner for the tool-capture rig.
* Same episode layout and CLI as mono_inertial_euroc, plus mav0/depth0/data:
* sparse ToF depth rendered into the camera frame by host/tof_depthmap.py,
* uint16 PNG in mm, named <ts_ns>.png like cam0. Depth exists only for camera
* frames that time-paired to a ToF frame; frames without a depth file are fed
* an all-zero depth image, so all their keypoints stay monocular (ORB-SLAM3
* treats depth<=0 per keypoint as "no depth").
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
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
#include<ctime>
#include<sstream>
#include<sys/stat.h>

#include<opencv2/core/core.hpp>

#include<System.h>
#include"ImuTypes.h"

using namespace std;

void LoadImages(const string &strImagePath, const string &strDepthPath,
                const string &strPathTimes, vector<string> &vstrImages,
                vector<string> &vstrDepth, vector<double> &vTimeStamps);

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps,
             vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro);

double ttrack_tot = 0;
int main(int argc, char *argv[])
{
    if(argc < 5)
    {
        cerr << endl << "Usage: ./rgbd_inertial_euroc path_to_vocabulary path_to_settings path_to_sequence_folder_1 path_to_times_file_1 (path_to_image_folder_2 path_to_times_file_2 ... path_to_image_folder_N path_to_times_file_N) " << endl;
        return 1;
    }

    const int num_seq = (argc-3)/2;
    cout << "num_seq = " << num_seq << endl;
    bool bFileName= (((argc-3) % 2) == 1);
    string file_name;
    if (bFileName)
    {
        file_name = string(argv[argc-1]);
        cout << "file name: " << file_name << endl;
    }

    // Load all sequences:
    int seq;
    vector< vector<string> > vstrImageFilenames;
    vector< vector<string> > vstrDepthFilenames;
    vector< vector<double> > vTimestampsCam;
    vector< vector<cv::Point3f> > vAcc, vGyro;
    vector< vector<double> > vTimestampsImu;
    vector<int> nImages;
    vector<int> nImu;
    vector<int> first_imu(num_seq,0);

    vstrImageFilenames.resize(num_seq);
    vstrDepthFilenames.resize(num_seq);
    vTimestampsCam.resize(num_seq);
    vAcc.resize(num_seq);
    vGyro.resize(num_seq);
    vTimestampsImu.resize(num_seq);
    nImages.resize(num_seq);
    nImu.resize(num_seq);

    int tot_images = 0;
    for (seq = 0; seq<num_seq; seq++)
    {
        cout << "Loading images for sequence " << seq << "...";

        string pathSeq(argv[(2*seq) + 3]);
        string pathTimeStamps(argv[(2*seq) + 4]);

        string pathCam0 = pathSeq + "/mav0/cam0/data";
        string pathDepth0 = pathSeq + "/mav0/depth0/data";
        string pathImu = pathSeq + "/mav0/imu0/data.csv";

        LoadImages(pathCam0, pathDepth0, pathTimeStamps,
                   vstrImageFilenames[seq], vstrDepthFilenames[seq], vTimestampsCam[seq]);
        cout << "LOADED!" << endl;

        int nDepth = 0;
        for(size_t i=0; i<vstrDepthFilenames[seq].size(); i++)
            if(!vstrDepthFilenames[seq][i].empty())
                nDepth++;
        cout << "Depth images present for " << nDepth << "/"
             << vstrDepthFilenames[seq].size() << " frames (others run monocular)." << endl;

        cout << "Loading IMU for sequence " << seq << "...";
        LoadIMU(pathImu, vTimestampsImu[seq], vAcc[seq], vGyro[seq]);
        cout << "LOADED!" << endl;

        nImages[seq] = vstrImageFilenames[seq].size();
        tot_images += nImages[seq];
        nImu[seq] = vTimestampsImu[seq].size();

        if((nImages[seq]<=0)||(nImu[seq]<=0))
        {
            cerr << "ERROR: Failed to load images or IMU for sequence" << seq << endl;
            return 1;
        }

        // Find first imu to be considered, supposing imu measurements start first

        while(vTimestampsImu[seq][first_imu[seq]]<=vTimestampsCam[seq][0])
            first_imu[seq]++;
        first_imu[seq]--; // first imu measurement to be considered

    }

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(tot_images);

    cout.precision(17);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    // [N6] Honor SLAM_NO_VIEWER so headless runs don't crash the Pangolin viewer under xvfb.
    bool bUseViewer = (getenv("SLAM_NO_VIEWER") == nullptr);
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_RGBD, bUseViewer);

    // [N6] Optional localization-only mode (freeze the loaded map and relocalize the
    // sequence into it, UMI-style). Gated on env ORBSLAM_LOCALIZATION_ONLY so the
    // EuRoC example's positional-arg signature stays unchanged.
    if (const char* loc = getenv("ORBSLAM_LOCALIZATION_ONLY"))
        if (std::string(loc) != "0" && std::string(loc).size()) {
            cout << "[N6] Localization-only mode ON (map frozen)." << endl;
            SLAM.ActivateLocalizationMode();
        }

    float imageScale = SLAM.GetImageScale();

    int proccIm=0;
    for (seq = 0; seq<num_seq; seq++)
    {
        // Main loop
        cv::Mat im, depth;
        cv::Mat depthZero;                 // reused all-zero depth for unpaired frames
        vector<ORB_SLAM3::IMU::Point> vImuMeas;
        proccIm = 0;
        for(int ni=0; ni<nImages[seq]; ni++, proccIm++)
        {
            // Read image from file
            im = cv::imread(vstrImageFilenames[seq][ni],cv::IMREAD_UNCHANGED);

            double tframe = vTimestampsCam[seq][ni];

            if(im.empty())
            {
                cerr << endl << "Failed to load image at: "
                     <<  vstrImageFilenames[seq][ni] << endl;
                return 1;
            }

            if(!vstrDepthFilenames[seq][ni].empty())
            {
                depth = cv::imread(vstrDepthFilenames[seq][ni], cv::IMREAD_UNCHANGED);
                if(depth.empty() || depth.type() != CV_16UC1)
                {
                    cerr << endl << "Bad depth image (want 16UC1) at: "
                         << vstrDepthFilenames[seq][ni] << endl;
                    return 1;
                }
            }
            else
            {
                if(depthZero.empty())
                    depthZero = cv::Mat::zeros(im.rows, im.cols, CV_16UC1);
                depth = depthZero;
            }

            if(imageScale != 1.f)
            {
                int width = im.cols * imageScale;
                int height = im.rows * imageScale;
                cv::resize(im, im, cv::Size(width, height));
                // Nearest neighbor: interpolating metric depth across the sparse
                // zone quads would manufacture edge depths the sensor never saw.
                cv::Mat dscaled;
                cv::resize(depth, dscaled, cv::Size(width, height), 0, 0, cv::INTER_NEAREST);
                depth = dscaled;
            }

            // Load imu measurements from previous frame
            vImuMeas.clear();

            if(ni>0)
            {
                while(vTimestampsImu[seq][first_imu[seq]]<=vTimestampsCam[seq][ni])
                {
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(vAcc[seq][first_imu[seq]].x,vAcc[seq][first_imu[seq]].y,vAcc[seq][first_imu[seq]].z,
                                                             vGyro[seq][first_imu[seq]].x,vGyro[seq][first_imu[seq]].y,vGyro[seq][first_imu[seq]].z,
                                                             vTimestampsImu[seq][first_imu[seq]]));
                    first_imu[seq]++;
                }
            }

    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
    #endif

            // Pass the image to the SLAM system
            SLAM.TrackRGBD(im,depth,tframe,vImuMeas);

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
            if(ni<nImages[seq]-1)
                T = vTimestampsCam[seq][ni+1]-tframe;
            else if(ni>0)
                T = tframe-vTimestampsCam[seq][ni-1];

            if(ttrack<T)
                usleep((T-ttrack)*1e6); // 1e6
        }
        if(seq < num_seq - 1)
        {
            cout << "Changing the dataset" << endl;

            SLAM.ChangeDataset();
        }
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    if (bFileName)
    {
        const string kf_file =  "kf_" + string(argv[argc-1]) + ".txt";
        const string f_file =  "f_" + string(argv[argc-1]) + ".txt";
        SLAM.SaveTrajectoryEuRoC(f_file);
        SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
    }
    else
    {
        SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
        SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    }

    return 0;
}

void LoadImages(const string &strImagePath, const string &strDepthPath,
                const string &strPathTimes, vector<string> &vstrImages,
                vector<string> &vstrDepth, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);
    vstrDepth.reserve(5000);
    struct stat st;
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
            string depthFile = strDepthPath + "/" + ss.str() + ".png";
            vstrDepth.push_back(stat(depthFile.c_str(), &st) == 0 ? depthFile : string());
            double t;
            ss >> t;
            vTimeStamps.push_back(t/1e9);
        }
    }
}

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps,
             vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro)
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
