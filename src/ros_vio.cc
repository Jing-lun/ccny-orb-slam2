#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include "poine_orbslam/Matrix.h"
#include <rosbag/bag.h>
#include "ctime"
#include "time.h"
#include "System.h"
#include "Converter.h"
#include "PangolinViewer.h"
#include "Viewer.h"
#include <boost/foreach.hpp>
#include <string>

using namespace std;
rosbag::Bag bag_record;
#if 1
string int2string(int value)
{
    stringstream ss;
    ss<<value;
    return ss.str();
}
#endif

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM, ros::NodeHandle nh):mpSLAM(pSLAM), nh_(nh){}

    void GrabRGBD_IMU(const sensor_msgs::ImageConstPtr& kinect2color_msg, const sensor_msgs::ImageConstPtr& kinect2depth_msg, const sensor_msgs::ImuConstPtr& imu_msg, const nav_msgs::OdometryConstPtr& odom_msg);
    
    ORB_SLAM2::System* mpSLAM;
	//ROS_ORB_SLAM::MapPublisher _map_pub;

    ros::NodeHandle nh_;
    ros::Publisher imu_pub;
    sensor_msgs::Imu imu_orb;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "VIORB");
  ros::Time::init();
  ros::start();
  string orbVocFile = "/home/robooster/viorb_config/config/ORBvoc.bin";
  string orbSetiingsFile = "/home/robooster/viorb_config/config/kinect2_sd.yaml";

  #if 1
  ORB_SLAM2::Viewer* viewer;

  viewer = new ORB_SLAM2::PangolinViewer(orbSetiingsFile);
  ORB_SLAM2::System orbslam( orbVocFile, orbSetiingsFile ,ORB_SLAM2::System::RGBD, viewer );
    //orbslam = new ORB_SLAM2::System( orbVocFile, orbSetiingsFile ,ORB_SLAM2::System::RGBD, viewer );
  #endif

  ros::NodeHandle nh;

  ImageGrabber igb(&orbslam, nh);

  message_filters::Subscriber<sensor_msgs::Image> kinect2color_sub(nh, "/kinect2/qhd/image_color_rect", 1);//订阅Kinect的Topic
  message_filters::Subscriber<sensor_msgs::Image> kinect2depth_sub(nh, "/kinect2/qhd/image_depth_rect", 1);//订阅Kinect的Topic
  message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh, "/imu_os3dm/imu_raw", 1);//订阅imu的Topic
  //message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh, "/imu/data", 1);
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "/mobile_base_controller/odom", 1);// 订阅odom的Topic
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Imu, nav_msgs::Odometry> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(20), kinect2color_sub, kinect2depth_sub, imu_sub, odom_sub);
  sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD_IMU, &igb, _1, _2, _3, _4));
  
  ofstream outfile("//home//robooster//Desktop//odometry_data.txt");
  //outfile<<"world_time"<<"\t\t  "<<"left"<<"\t\t\t\t"<<"front"<<"\t\t\t\t"<<"above"<<endl;
  ofstream outfile1("//home//robooster//Desktop//orbslam_data.txt");
  //outfile1<<"world_time"<<"\t\t  "<<"q_x"<<"\t\t\t\t"<<"q_y"<<"\t\t\t\t"<<"q_z"<<"\t\t\t\t"<<"q_w"<<"\t\t\t\t"<<"left"<<"\t\t\t\t"<<"front"<<"\t\t\t\t"<<"above"<<"\t\t\t\t"<<endl;
  //outfile1<<"world_time"<<"\t\t  "<<"q_x"<<"\t\t\t\t"<<"q_y"<<"\t\t\t\t"<<"q_z"<<"\t\t\t\t"<<"q_w"<<"\t\t\t\t"<<"x_q"<<"\t\t\t\t"<<"y_q"<<"\t\t\t\t"<<"z_q"<<"\t\t\t\t"<<"x_m"<<"\t\t\t\t"<<"y_m"<<"\t\t\t\t"<<"z_m"<<"\t\t\t\t"<<"left"<<"\t\t\t\t"<<"front"<<"\t\t\t\t"<<"above"<<"\t\t\t\t"<<endl;
  ofstream outfile2("//home//robooster//Desktop//imu_data_raw.txt");
  //ofstream outfile2("//home//robooster//Desktop//imu_data_orb.txt");
  //outfile2<<"world_time"<<"\t\t  "<<"q_x"<<"\t\t\t\t"<<"q_y"<<"\t\t\t\t"<<"q_z"<<"\t\t\t\t"<<"q_w"<<endl;
  //outfile2<<"world_time"<<"\t\t  "<<"acc_x"<<"\t\t\t "<<"acc_y"<<"\t\t\t "<<"acc_z"<<"\t\t\t ";
  //outfile2<<"mag_x"<<"\t\t\t "<<"mag_y"<<"\t\t\t "<<"mag_z"<<"\t\t\t ";
  //outfile2<<"gyro_x"<<"\t\t\t "<<"gyro_y"<<"\t\t\t "<<"gyro_z"<<"\t\t\t "<<endl;   
  ofstream outfile3("//home//robooster//Desktop//imu_data_for_orb.txt"); 
  //outfile3<<"world_time"<<"\t\t  "<<"x"<<"\t\t\t\t"<<"y"<<"\t\t\t\t"<<"z"<<endl;

  ofstream outfile4("//home//robooster//Desktop//imucallback.txt"); 
  ofstream outfile5("//home//robooster//Desktop//orbcallback.txt"); 

  ros::spin();
  const string map_trajectory = "/home/robooster/viorb_config/trajectory/map_trajectory";
  orbslam.SaveKeyFrameTrajectoryTUM(map_trajectory);
  orbslam.Shutdown();
  ros::shutdown();
  return 0;
}

void ImageGrabber::GrabRGBD_IMU(const sensor_msgs::ImageConstPtr& kinect2color_msg, const sensor_msgs::ImageConstPtr& kinect2depth_msg, const sensor_msgs::ImuConstPtr& imu_msg, const nav_msgs::OdometryConstPtr& odom_msg)
{
    // IMU 数据保存
    #if 1
    ofstream outfile2("//home//robooster//Desktop//imu_data_raw.txt", ios_base::app);
    Eigen::Quaterniond q;
    Eigen::Matrix<double,3,1> imu_w;         // IMU的角速度，测量值
    Eigen::Matrix<double,3,1> imu_a;         // IMU的线加速度，测量值
    q.x() = imu_msg->orientation.x;
    q.y() = imu_msg->orientation.y;
    q.z() = imu_msg->orientation.z;
    q.w() = imu_msg->orientation.w;
    imu_w << imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z;
    imu_a << imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z;
    outfile2.setf(ios::fixed);
    outfile2<< fixed << setprecision(5) << std::left << std::setw(18)<<imu_msg->header.stamp.now().toSec();
    outfile2<< fixed << setprecision(8) << std::left << std::setw(15)<<q.x();
    outfile2<< fixed << setprecision(8) << std::left << std::setw(15)<<q.y();
    outfile2<< fixed << setprecision(8) << std::left << std::setw(15)<<q.z();
    outfile2<< fixed << setprecision(8) << std::left << std::setw(15)<<q.w()<<endl;
    #endif

    // Odom 数据保存
    #if 1
    double px, py, pz;      
    Eigen::Matrix<double,3,1> odom;         
    // pz = odom_msg->pose.pose.position.x;
    // px = odom_msg->pose.pose.position.y;
    // py = odom_msg->pose.pose.position.z;
    pz = -odom_msg->pose.pose.position.z;
    px = odom_msg->pose.pose.position.x;
    py = -odom_msg->pose.pose.position.y;
    odom << py, pz, px;
    ofstream outfile("//home//robooster//Desktop//odometry_data.txt", ios_base::app);
    outfile.setf(ios::fixed);
    outfile<< fixed << setprecision(5) << std::left << std::setw(18)<<odom_msg->header.stamp.now().toSec();
    outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<py;
    outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<pz;
    outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<px<<endl;
    #endif 

    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(kinect2color_msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(kinect2depth_msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //把imu直接输出的四元数、加计、陀螺数据当做参数传递
    mpSLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, kinect2color_msg->header.stamp.toSec(), q, imu_a, imu_w, odom);
    //cv::Mat cam_pose = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
    
    //_map_pub.PublishCurrentCamera(cam_pose);
}

