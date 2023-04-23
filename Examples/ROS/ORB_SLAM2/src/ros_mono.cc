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
#include<unordered_map>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>

#include"../../../include/KeyFrame.h"
#include"../../../include/System.h"
#include"../../../include/Map.h"
#include"../../../include/MapPoint.h"

#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    void publish(ros::Publisher pub_keyframes, ros::Publisher pub_keypoints, ros::Publisher pub_mappoints);
    ORB_SLAM2::System* mpSLAM;
};

void ImageGrabber::publish(ros::Publisher pub_keyframes, ros::Publisher pub_keypoints, ros::Publisher pub_mappoints) {
    ORB_SLAM2::Map* map = mpSLAM->mpMap;
    if (!map) {
        return;
    }
    const vector<ORB_SLAM2::KeyFrame*> keyframes = map->GetAllKeyFrames();
    int m = keyframes.size();
    if (!m) {
        return;
    }
    const vector<ORB_SLAM2::MapPoint*> mappoints = map->GetAllMapPoints();
    int n = mappoints.size();
    if (!n) {
        return;
    }
    unordered_map<ORB_SLAM2::MapPoint*, int> mappoints_index;
    for (int j = 0; j < n; j++) {
        ORB_SLAM2::MapPoint* map_point = mappoints[j];
        if (!map_point || map_point->isBad()) {
            continue;
        }
        mappoints_index[map_point] = j;
    }
    int rows = 480;
    int cols = 640;
    int channels = 3;
    std_msgs::UInt8MultiArray keyframes_msg;
    keyframes_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    keyframes_msg.layout.dim[0].label = "j";
    keyframes_msg.layout.dim[0].size = m;
    keyframes_msg.layout.dim[0].stride = m * rows * cols * channels;
    keyframes_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    keyframes_msg.layout.dim[1].label = "height";
    keyframes_msg.layout.dim[1].size = rows;
    keyframes_msg.layout.dim[1].stride = rows * cols * channels;
    keyframes_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    keyframes_msg.layout.dim[2].label = "width";
    keyframes_msg.layout.dim[2].size = cols;
    keyframes_msg.layout.dim[2].stride = cols * channels;
    keyframes_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    keyframes_msg.layout.dim[3].label = "channel";
    keyframes_msg.layout.dim[3].size = channels;
    keyframes_msg.layout.dim[3].stride = channels;
    keyframes_msg.data.resize(keyframes_msg.layout.dim[0].stride);

    std_msgs::Float32MultiArray mappoints_msg;
    mappoints_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    mappoints_msg.layout.dim[0].label = "j";
    mappoints_msg.layout.dim[0].size = n;
    mappoints_msg.layout.dim[0].stride = n * 3;
    mappoints_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    mappoints_msg.layout.dim[1].label = "map_point";
    mappoints_msg.layout.dim[1].size = 3;
    mappoints_msg.layout.dim[1].stride = 3;
    mappoints_msg.data.resize(mappoints_msg.layout.dim[0].stride);

    for (int j = 0; j < n; j++) {
        ORB_SLAM2::MapPoint* map_point = mappoints[j];
        if (!map_point || map_point->isBad()) {
            continue;
        }
        cv::Mat pos = map_point->GetWorldPos();
        mappoints_msg.data[j * 3] = pos.at<float>(0);
        mappoints_msg.data[j * 3 + 1] = pos.at<float>(1);
        mappoints_msg.data[j * 3 + 2] = pos.at<float>(2);
    }

    // Hard coded max of 1000 keypoints per keyframe
    int o = 1000;
    std_msgs::Float32MultiArray keypoints_msg;
    keypoints_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    keypoints_msg.layout.dim[0].label = "j";
    keypoints_msg.layout.dim[0].size = m;
    keypoints_msg.layout.dim[0].stride = m * o * 3;
    keypoints_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    keypoints_msg.layout.dim[1].label = "k";
    keypoints_msg.layout.dim[1].size = o;
    keypoints_msg.layout.dim[1].stride = o * 3;
    keypoints_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    keypoints_msg.layout.dim[2].label = "keypoint";
    keypoints_msg.layout.dim[2].size = 3;
    keypoints_msg.layout.dim[2].stride = 3;
    keypoints_msg.data.resize(keypoints_msg.layout.dim[0].stride);
    for (int j = 0; j < m; j++) {
        ORB_SLAM2::KeyFrame* keyframe = keyframes[j];
        if (!keyframe) {
            continue;
        }
        cv::Mat im = keyframe->im;
        if (im.empty()) {
            continue;
        }
        for (int a = 0; a < rows; a++) {
            for (int b = 0; b < cols; b++) {
                for (int c = 0; c < channels; c++) {
                    keyframes_msg.data[j * rows * cols * channels + a * cols * channels + b * channels + c] = im.at<cv::Vec3b>(a, b)[c];
                }
            }
        }
        std::vector<cv::KeyPoint> keypoints = keyframe->mvKeys;
        std::vector<ORB_SLAM2::MapPoint*> matches = keyframe->GetMapPointMatches();
        for (unsigned int k = 0; k < min(static_cast<std::vector<int>::size_type>(o), keypoints.size()); k++) {
            cv::KeyPoint keypoint = keypoints[k];
            ORB_SLAM2::MapPoint* match = matches[k];
            if (!match) {
                continue;
            }
            keypoints_msg.data[j * o * 3 + k * 3 + 0] = keypoint.pt.x;
            keypoints_msg.data[j * o * 3 + k * 3 + 1] = keypoint.pt.y;
            keypoints_msg.data[j * o * 3 + k * 3 + 2] = mappoints_index[match];
        }
    }
    pub_keyframes.publish(keyframes_msg);
    pub_keypoints.publish(keypoints_msg);
    pub_mappoints.publish(mappoints_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler("orb_slam2");
    ros::Publisher pub_keyframes = nodeHandler.advertise<std_msgs::UInt8MultiArray>("keyframes", 1);
    ros::Publisher pub_keypoints = nodeHandler.advertise<std_msgs::Float32MultiArray>("keypoints", 1);
    ros::Publisher pub_mappoints = nodeHandler.advertise<std_msgs::Float32MultiArray>("mappoints", 1);
    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);

    int duration = 10;
    ros::Timer timer = nodeHandler.createTimer(
        ros::Duration(duration),
        boost::bind(&ImageGrabber::publish, &igb, pub_keyframes, pub_keypoints, pub_mappoints)
    );
    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
}
