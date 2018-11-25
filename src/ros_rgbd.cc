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

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"System.h"
  #include "Converter.h"
  #include "pointcloudmapping.h"
  #include "PangolinViewer.h"
  #include "Viewer.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    //ORB_SLAM2::System* orbslam = nullptr;
    string orbVocFile = "/home/jinglun/viorb_config/config/ORBvoc.bin";
    string orbSetiingsFile = "/home/jinglun/viorb_config/config/kinect2_sd.yaml";

    #if 1
    ORB_SLAM2::Viewer* viewer;

    viewer = new ORB_SLAM2::PangolinViewer(orbSetiingsFile);
	ORB_SLAM2::System orbslam( orbVocFile, orbSetiingsFile ,ORB_SLAM2::System::RGBD, viewer );
    //orbslam = new ORB_SLAM2::System( orbVocFile, orbSetiingsFile ,ORB_SLAM2::System::RGBD, viewer );
    #endif

    ImageGrabber igb(&orbslam);

    ros::NodeHandle nh;

    //message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
    //message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth_registered/image_raw", 1);
	message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/kinect2/qhd/image_color_rect", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/kinect2/qhd/image_depth_rect", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    //ofstream outfile("//home//robooster//Desktop//odometry_data.txt");
    //outfile<<"world_time"<<"\t\t  "<<"left"<<"\t\t\t\t"<<"front"<<"\t\t\t\t"<<"above"<<endl;
    //ofstream outfile1("//home//robooster//Desktop//orbslam_data.txt");
    //outfile1<<"world_time"<<"\t\t  "<<"q_x"<<"\t\t\t\t"<<"q_y"<<"\t\t\t\t"<<"q_z"<<"\t\t\t\t"<<"q_w"<<"\t\t\t\t"<<"left"<<"\t\t\t\t"<<"front"<<"\t\t\t\t"<<"above"<<"\t\t\t\t"<<endl;
    //ofstream outfile2("//home//robooster//Desktop//imu_data_raw.txt");
    //outfile2<<"world_time"<<"\t\t  "<<"q_x"<<"\t\t\t\t"<<"q_y"<<"\t\t\t\t"<<"q_z"<<"\t\t\t\t"<<"q_w"<<endl;

    ros::spin();

    // Stop all threads
    orbslam.Shutdown();

    // Save camera trajectory
    //SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    //cout<<"can we call back?"<<endl;
    mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
    //cout<<"can we call back?"<<endl;
}


