/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>
#include <unistd.h>

#include<opencv2/core/core.hpp>

#include"System.h"

using namespace std;

void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames, const string &strSemantic, vector<string> &vstrSemanticFile,
                vector<double> &vTimestamps);
void LoadMask(const string &strFilenamesMask, cv::Mat &imMask);


int main(int argc, char **argv)
{
    if(argc != 5)
    {
        cerr << endl << "Usage: ./mono_kitti path_to_vocabulary path_to_settings path_to_sequence path_to_segment" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<string> vstrSemanticFile;
    vector<double> vTimestamps;
    LoadImages(string(argv[3]), vstrImageFilenames, string(argv[4]), vstrSemanticFile, vTimestamps);

    int nImages = vstrImageFilenames.size();

    // 12.21 验证图片和语义数量是否正确
    if(vstrImageFilenames.empty())
    {
        cerr << endl << "error1: No images found in provided path." << endl;
        return 1;
    }
    else if(vstrSemanticFile.empty())
    {
        cerr << endl << "error2: No semanticFile found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenames.size() != vstrSemanticFile.size())
    {
        cerr << endl << "error3: Different number of images for segmentation." << endl;
        return 1;
    }


    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    // Vector for tracking time statistics
    // 存储每一帧追踪所需的时间
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat im;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: " << vstrImageFilenames[ni] << endl;
            return 1;
        }


#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif
        // 12.21
        cv::Mat imSem(im.rows, im.cols, CV_32SC1);
        LoadMask(vstrSemanticFile[ni], im);
        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,tframe, imSem);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        // 计算追踪该图片花的时间
        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        // 计算下一帧图片的时间戳与当前时间戳的差值T,与追踪所需时间进行比较　tframe = vTimestamps[ni];
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];
        // 如果有必要，停止当前线程
        // 模拟时间情况，因为追踪结束后下一帧可能还没来
        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    // 追踪完所有图片，关闭线程
    SLAM.Shutdown();

    // Tracking time statistics
    // 统计追踪部分，计算中位数　总数　平均数
    // 对所有帧的追踪时间进行一个排序
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    // 保存整个相机的位姿轨迹
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");    

    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, const string &strPathToSemantic, vector<string> &vstrSemanticFile, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    // 12.21 
    fTimes.close();
    string strPrefixLeft = strPathToSequence + "/image_0/";
    string strPrefixSemantic = strPathToSemantic + "/image_0/";

    const int nTimes = vTimestamps.size();
    vstrImageFilenames.resize(nTimes);
    vstrSemanticFile.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
        vstrSemanticFile[i] = strPrefixSemantic + ss.str() + ".txt";
    }
}

void LoadMask(const string &strFilenamesMask, cv::Mat &im)
{
    ifstream file_mask;
    file_mask.open(strFilenamesMask.c_str());

    // Main loop
    // count代表图片的行数
    int count = 0;
    // ｉｍｇＬａｂｅｌ是为了展示
    // cv::Mat imgLabel(im.rows,im.cols,CV_8UC3); // for display
    while(!file_mask.eof())
    {
        string s;
        getline(file_mask, s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            int tmp;
            // 根据掩码图片的矩阵列宽，遍历读取每个变量
            for(int i = 0; i < im.cols; ++i){
                ss >> tmp;
                if (tmp!=0){
                    im.at<uchar>(count,i) = 128;
                    // 查看到属于某个实例的像素时，
                    // 给这个像素点对应的实例图片位置处赋值
                    // im.at<int>(count,i) = tmp;
                    // 根据tmp的值，给要作为展示的图片，赋值不同的颜色
                    // if (tmp>50)
                    //     tmp = tmp/2;
                    // switch (tmp)
                    // {
                    //     case 0:
                    //         im.at<cv::Vec3b>(count,i) = cv::Vec3b(0,255,255);
                    //         break;
                    //     case 1:
                    //         im.at<cv::Vec3b>(count,i) = cv::Vec3b(0,0,255);  // red
                    //         break;
                    //     case 2:
                    //         im.at<cv::Vec3b>(count,i) = cv::Vec3b(255,0,0);  // blue
                    //         break;
                    //     case 3:
                    //         im.at<cv::Vec3b>(count,i) = cv::Vec3b(255,255,0); // cyan
                    //         break;
                    //     case 4:
                    //         im.at<cv::Vec3b>(count,i) = cv::Vec3b(47,255,173); // green yellow
                    //         break;
                    //     case 5:
                    //         im.at<cv::Vec3b>(count,i) = cv::Vec3b(128, 0, 128);
                    //         break;
                    //     case 6:
                    //         im.at<cv::Vec3b>(count,i) = cv::Vec3b(203,192,255);
                    //         break;
                    //     case 7:
                    //         im.at<cv::Vec3b>(count,i) = cv::Vec3b(196,228,255);
                    //         break;
                    //     case 8:
                    //         im.at<cv::Vec3b>(count,i) = cv::Vec3b(42,42,165);
                    //         break;
                    //     case 9:
                    //         im.at<cv::Vec3b>(count,i) = cv::Vec3b(255,255,255);
                    //         break;
                    //     case 10:
                    //         im.at<cv::Vec3b>(count,i) = cv::Vec3b(245,245,245); // whitesmoke
                    //         break;
                    //     case 11:
                    //         im.at<cv::Vec3b>(count,i) = cv::Vec3b(0,165,255); // orange
                    //         break;
                    //     case 12:
                    //         im.at<cv::Vec3b>(count,i) = cv::Vec3b(230,216,173); // lightblue
                    //         break;
                    //     case 13:
                    //         im.at<cv::Vec3b>(count,i) = cv::Vec3b(128,128,128); // grey
                    //         break;
                    //     case 14:
                    //         im.at<cv::Vec3b>(count,i) = cv::Vec3b(0,215,255); // gold
                    //         break;
                    //     case 15:
                    //         im.at<cv::Vec3b>(count,i) = cv::Vec3b(30,105,210); // chocolate
                    //         break;
                    //     case 16:
                    //         im.at<cv::Vec3b>(count,i) = cv::Vec3b(0,255,0);  // green
                    //         break;
                    //     case 17:
                    //         im.at<cv::Vec3b>(count,i) = cv::Vec3b(34, 34, 178);  // firebrick
                    //         break;
                    //     case 18:
                    //         im.at<cv::Vec3b>(count,i) = cv::Vec3b(240, 255, 240);  // honeydew
                    //         break;
                    //     case 19:
                    //         im.at<cv::Vec3b>(count,i) = cv::Vec3b(250, 206, 135);  // lightskyblue
                    //         break;
                    //     case 20:
                    //         im.at<cv::Vec3b>(count,i) = cv::Vec3b(238, 104, 123);  // mediumslateblue
                    //         break;
                    //     case 21:
                    //         im.at<cv::Vec3b>(count,i) = cv::Vec3b(225, 228, 255);  // mistyrose
                    //         break;
                    //     case 22:
                    //         im.at<cv::Vec3b>(count,i) = cv::Vec3b(128, 0, 0);  // navy
                    //         break;
                    //     case 23:
                    //         im.at<cv::Vec3b>(count,i) = cv::Vec3b(35, 142, 107);  // olivedrab
                    //         break;
                    //     case 24:
                    //         im.at<cv::Vec3b>(count,i) = cv::Vec3b(45, 82, 160);  // sienna
                    //         break;
                    //     case 25:
                    //         im.at<cv::Vec3b>(count,i) = cv::Vec3b(0, 255, 127); // chartreuse
                    //         break;
                    //     case 26:
                    //         im.at<cv::Vec3b>(count,i) = cv::Vec3b(139, 0, 0);  // darkblue
                    //         break;
                    //     case 27:
                    //         im.at<cv::Vec3b>(count,i) = cv::Vec3b(60, 20, 220);  // crimson
                    //         break;
                    //     case 28:
                    //         im.at<cv::Vec3b>(count,i) = cv::Vec3b(0, 0, 139);  // darkred
                    //         break;
                    //     case 29:
                    //         im.at<cv::Vec3b>(count,i) = cv::Vec3b(211, 0, 148);  // darkviolet
                    //         break;
                    //     case 30:
                    //         im.at<cv::Vec3b>(count,i) = cv::Vec3b(255, 144, 30);  // dodgerblue
                    //         break;
                    //     case 31:
                    //         im.at<cv::Vec3b>(count,i) = cv::Vec3b(105, 105, 105);  // dimgray
                    //         break;
                    //     case 32:
                    //         im.at<cv::Vec3b>(count,i) = cv::Vec3b(180, 105, 255);  // hotpink
                    //         break;
                    //     case 33:
                    //         im.at<cv::Vec3b>(count,i) = cv::Vec3b(204, 209, 72);  // mediumturquoise
                    //         break;
                    //     case 34:
                    //         im.at<cv::Vec3b>(count,i) = cv::Vec3b(173, 222, 255);  // navajowhite
                    //         break;
                    //     case 35:
                    //         im.at<cv::Vec3b>(count,i) = cv::Vec3b(143, 143, 188); // rosybrown
                    //         break;
                    //     case 36:
                    //         im.at<cv::Vec3b>(count,i) = cv::Vec3b(50, 205, 50);  // limegreen
                    //         break;
                    //     case 37:
                    //         im.at<cv::Vec3b>(count,i) = cv::Vec3b(34, 34, 178);  // firebrick
                    //         break;
                    //     case 38:
                    //         im.at<cv::Vec3b>(count,i) = cv::Vec3b(240, 255, 240);  // honeydew
                    //         break;
                    //     case 39:
                    //         im.at<cv::Vec3b>(count,i) = cv::Vec3b(250, 206, 135);  // lightskyblue
                    //         break;
                    //     case 40:
                    //         im.at<cv::Vec3b>(count,i) = cv::Vec3b(238, 104, 123);  // mediumslateblue
                    //         break;
                    //     case 41:
                    //         im.at<cv::Vec3b>(count,i) = cv::Vec3b(225, 228, 255);  // mistyrose
                    //         break;
                    //     case 42:
                    //         im.at<cv::Vec3b>(count,i) = cv::Vec3b(128, 0, 0);  // navy
                    //         break;
                    //     case 43:
                    //         im.at<cv::Vec3b>(count,i) = cv::Vec3b(35, 142, 107);  // olivedrab
                    //         break;
                    //     case 44:
                    //         im.at<cv::Vec3b>(count,i) = cv::Vec3b(45, 82, 160);  // sienna
                    //         break;
                    //     case 45:
                    //         im.at<cv::Vec3b>(count,i) = cv::Vec3b(30,105,210); // chocolate
                    //         break;
                    //     case 46:
                    //         im.at<cv::Vec3b>(count,i) = cv::Vec3b(0,255,0);  // green
                    //         break;
                    //     case 47:
                    //         im.at<cv::Vec3b>(count,i) = cv::Vec3b(34, 34, 178);  // firebrick
                    //         break;
                    //     case 48:
                    //         im.at<cv::Vec3b>(count,i) = cv::Vec3b(240, 255, 240);  // honeydew
                    //         break;
                    //     case 49:
                    //         im.at<cv::Vec3b>(count,i) = cv::Vec3b(250, 206, 135);  // lightskyblue
                    //         break;
                    //     case 50:
                    //         im.at<cv::Vec3b>(count,i) = cv::Vec3b(238, 104, 123);  // mediumslateblue
                    //         break;
                    // }
                }
                // else{
                //     im.at<int>(count,i) = 0;
                //     imgLabel.at<cv::Vec3b>(count,i) = cv::Vec3b(255,255,240); // azure
                // }
                // cout << imMask.at<int>(count,i) << " ";
            }
            // cout << endl;
            count++;
        }
    }

    // // Display the img_mask
    // cv::imshow("Segmentation Mask", imgLabel);
    // int i = 0;
    // for(int j = 0; j < 5000; j++) {
    //     i++;
    // }

    return;
}
