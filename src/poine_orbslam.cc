  #include <stdlib.h>
  #include <stdio.h>
  #include <iostream>
  #include <sstream>
  #include <string>
  #include <vector>
  #include <cmath>
  #include <mutex>
  #include <thread>
  #include <chrono>
  #include <ros/ros.h>
  #include <ros/spinner.h>
  #include <sensor_msgs/CameraInfo.h>
  #include <sensor_msgs/Image.h>
  #include <sensor_msgs/PointCloud.h>
  #include <cv_bridge/cv_bridge.h>
  #include <fstream>
  #include <algorithm>
  #include <image_transport/image_transport.h>
  #include <image_transport/subscriber_filter.h>
<<<<<<< HEAD
  #include "poine_orbslam/Matrix.h"
=======
>>>>>>> d44933e8356f6014b6c205e747bf5c9be8dd54d6
  
  #include <message_filters/subscriber.h>
  #include <message_filters/synchronizer.h>
  #include <message_filters/sync_policies/exact_time.h>
  #include <message_filters/sync_policies/approximate_time.h>
  
  #include <kinect2_bridge/kinect2_definitions.h>

  #include "System.h"
<<<<<<< HEAD
  #include "Converter.h"
=======
>>>>>>> d44933e8356f6014b6c205e747bf5c9be8dd54d6
  #include "pointcloudmapping.h"
  #include "PangolinViewer.h"
  #include "Viewer.h"
  #include <signal.h>  
  #include <geometry_msgs/Twist.h> 
<<<<<<< HEAD
  #include <nav_msgs/Odometry.h>
  #include <sensor_msgs/Imu.h>
  // #include "imu_os3dm/imu.h"
  #include <math.h>
  #include <fstream>

  //void FrameTcwCallback( const poine_orbslam::Matrix::ConstPtr& msg)
  // {
  //   ROS_INFO("I heard: msg: Matrix %f",msg->Frame_mTcw);
  // }

  void BaseCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
      static double px, py, pz;               
      pz = msg->pose.pose.position.x;
      px = msg->pose.pose.position.y;
      py = msg->pose.pose.position.z;
      ofstream outfile("//home//robooster//Desktop//Odometry_data.txt", ios_base::app);
      outfile.setf(ios::fixed);
      double secs =ros::Time::now().toSec();
      outfile<< fixed << setprecision(3) << std::left << std::setw(18)<<secs;
      outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<px;
      outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<pz;
      outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<py<<endl;
  }

  #if 0 // imu_raw
  void IMUCallback( const sensor_msgs::Imu::ConstPtr& msg )
  {
     #if 1
     //static bool flag = true;
     Eigen::Quaterniond q;
     //static Eigen::Quaterniond q0;
     q.x() = msg->orientation.x;
     q.y() = msg->orientation.y;
     q.z() = msg->orientation.z;
     q.w() = msg->orientation.w;
     #endif

     #if 0
     cout<<"q.x() before= "<<q.x()<<endl;
     cout<<"q.y() before= "<<q.y()<<endl;
     cout<<"q.z() before= "<<q.z()<<endl;
     cout<<"q.w() before= "<<q.w()<<endl;
     #endif

     #if 0
     if(flag)
     {
     q0.x() = 0.0;
     q0.y() = 0.0;
     q0.z() = 0.0;
     q0.w() = 0.0;
     q0=q.inverse();

     if( abs(q0.x()-0.0)>1e-5 || abs(q0.y()-0.0)>1e-5 || abs(q0.z()-0.0)>1e-5 || abs(q0.w()-0.0)>1e-5 )
      {
        flag = false;
      }
     else
      {
        flag = true;
      }
     }
     #endif

     #if 0
     cout<<"q0.x() = "<<q0.x()<<endl;
     cout<<"q0.y() = "<<q0.y()<<endl;
     cout<<"q0.z() = "<<q0.z()<<endl;
     cout<<"q0.w() = "<<q0.w()<<endl;
     #endif

     //q = q*q0;

     #if 0
     cout<<"q.x() after= "<<q.x()<<endl;
     cout<<"q.y() after= "<<q.y()<<endl;
     cout<<"q.z() after= "<<q.z()<<endl;
     cout<<"q.w() after= "<<q.w()<<endl;
     #endif

    #if 0      
    static double gyro_x, gyro_y, gyro_z;
    static double acc_x, acc_y, acc_z;
    static double mag_x, mag_y, mag_z;
        
    acc_x = msg->Acc.x;
    acc_y = msg->Acc.y;
    acc_z = msg->Acc.z;
    mag_x = msg->Mag.x;
    mag_y = msg->Mag.y;
    mag_z = msg->Mag.z;
    gyro_x = msg->Gyro.x;
    gyro_y = msg->Gyro.y;
    gyro_z = msg->Gyro.z;
    #endif

    ofstream outfile("//home//robooster//Desktop//imu_data_raw.txt", ios_base::app);
    outfile.setf(ios::fixed);
    double secs =ros::Time::now().toSec();
    outfile<< fixed << setprecision(3) << std::left << std::setw(18)<<secs;

    #if 1
    outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<q.x();
    outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<q.y();
    outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<q.z();
    outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<q.w()<<endl;
    #endif

    #if 0
    outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<acc_x;
    outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<acc_y;
    outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<acc_z;
    outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<mag_x;
    outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<mag_y;
    outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<mag_z;
    outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<gyro_x;
    outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<gyro_y;
    outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<gyro_z<<endl;
    #endif      

  }
  #endif
  
  #if 0 // imu_orb
  void IMUCallback( const imu_os3dm::imu::ConstPtr& msg )
  {
     #if 0
     //static bool flag = true;
     Eigen::Quaterniond q;
     //static Eigen::Quaterniond q0;
     q.x() = msg->orientation.x;
     q.y() = msg->orientation.y;
     q.z() = msg->orientation.z;
     q.w() = msg->orientation.w;
     #endif

     #if 0
     cout<<"q.x() before= "<<q.x()<<endl;
     cout<<"q.y() before= "<<q.y()<<endl;
     cout<<"q.z() before= "<<q.z()<<endl;
     cout<<"q.w() before= "<<q.w()<<endl;
     #endif

     #if 0
     if(flag)
     {
     q0.x() = 0.0;
     q0.y() = 0.0;
     q0.z() = 0.0;
     q0.w() = 0.0;
     q0=q.inverse();

     if( abs(q0.x()-0.0)>1e-5 || abs(q0.y()-0.0)>1e-5 || abs(q0.z()-0.0)>1e-5 || abs(q0.w()-0.0)>1e-5 )
      {
        flag = false;
      }
     else
      {
        flag = true;
      }
     }
     #endif

     #if 0
     cout<<"q0.x() = "<<q0.x()<<endl;
     cout<<"q0.y() = "<<q0.y()<<endl;
     cout<<"q0.z() = "<<q0.z()<<endl;
     cout<<"q0.w() = "<<q0.w()<<endl;
     #endif

     //q = q*q0;

     #if 0
     cout<<"q.x() after= "<<q.x()<<endl;
     cout<<"q.y() after= "<<q.y()<<endl;
     cout<<"q.z() after= "<<q.z()<<endl;
     cout<<"q.w() after= "<<q.w()<<endl;
     #endif

    #if 1      
    static double gyro_x, gyro_y, gyro_z;
    static double acc_x, acc_y, acc_z;
    static double mag_x, mag_y, mag_z;
        
    acc_x = msg->Acc.x;
    acc_y = msg->Acc.y;
    acc_z = msg->Acc.z;
    mag_x = msg->Mag.x;
    mag_y = msg->Mag.y;
    mag_z = msg->Mag.z;
    gyro_x = msg->Gyro.x;
    gyro_y = msg->Gyro.y;
    gyro_z = msg->Gyro.z;
    #endif

    ofstream outfile("//home//robooster//Desktop//imu_data_orb.txt", ios_base::app);
    outfile.setf(ios::fixed);
    double secs =ros::Time::now().toSec();
    outfile<< fixed << setprecision(5) << std::left << std::setw(20)<<secs;
    #if 0
    outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<q.x();
    outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<q.y();
    outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<q.z();
    outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<q.w()<<endl;
    #endif

    #if 1
    outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<acc_x;
    outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<acc_y;
    outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<acc_z;
    outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<mag_x;
    outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<mag_y;
    outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<mag_z;
    outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<gyro_x;
    outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<gyro_y;
    outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<gyro_z<<endl;
    #endif      

  }
  #endif

  string getTime()
  {
      time_t timep;
      time (&timep);
      char tmp[64];
      strftime(tmp, sizeof(tmp), "%Y-%m-%d_%H:%M:%S",localtime(&timep) );
      return tmp;
  }
=======
  ros::Publisher cmdVelPub;

  void shutdown(int sig)  
  {
    ros::shutdown();  
  } 
>>>>>>> d44933e8356f6014b6c205e747bf5c9be8dd54d6

  class Receiver
  {
  public:
    enum Mode
    {
      IMAGE = 0,
      CLOUD,
      BOTH
    };
private:
    std::mutex lock;
  
    const std::string topicColor, topicDepth;
    const bool useExact, useCompressed;
  
    bool updateImage, updateCloud;
    bool save;
    bool running;
    size_t frame;
    const size_t queueSize;
  
    cv::Mat color, depth;
    cv::Mat cameraMatrixColor, cameraMatrixDepth;
    cv::Mat lookupX, lookupY;
  
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactSyncPolicy;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximateSyncPolicy;
  
    ros::NodeHandle nh;
    ros::Publisher cloud_pub=nh.advertise<sensor_msgs::PointCloud>("cloud",50);
    ros::AsyncSpinner spinner;
    image_transport::ImageTransport it;
    image_transport::SubscriberFilter *subImageColor, *subImageDepth;
    message_filters::Subscriber<sensor_msgs::CameraInfo> *subCameraInfoColor, *subCameraInfoDepth;
  
    message_filters::Synchronizer<ExactSyncPolicy> *syncExact;
    message_filters::Synchronizer<ApproximateSyncPolicy> *syncApproximate;
  
    std::thread imageViewerThread;
    Mode mode;
  
    std::ostringstream oss;
    std::vector<int> params;
  
    //RGBDSLAM  slam; //the slam object
    ORB_SLAM2::System* orbslam    =nullptr;
<<<<<<< HEAD
    string tat;
=======
>>>>>>> d44933e8356f6014b6c205e747bf5c9be8dd54d6

 public:
    Receiver(const std::string &topicColor, const std::string &topicDepth, const bool useExact, const bool useCompressed)
      : topicColor(topicColor), topicDepth(topicDepth), useExact(useExact), useCompressed(useCompressed),
        updateImage(false), updateCloud(false), save(false), running(false), frame(0), queueSize(5),
        nh("~"), spinner(0), it(nh), mode(CLOUD)
    {
      cameraMatrixColor = cv::Mat::zeros(3, 3, CV_64F);
      cameraMatrixDepth = cv::Mat::zeros(3, 3, CV_64F);
      params.push_back(cv::IMWRITE_JPEG_QUALITY);
      params.push_back(100);
      params.push_back(cv::IMWRITE_PNG_COMPRESSION);
      params.push_back(1);
      params.push_back(cv::IMWRITE_PNG_STRATEGY);
      params.push_back(cv::IMWRITE_PNG_STRATEGY_RLE);
      params.push_back(0);
  
<<<<<<< HEAD
      string orbVocFile = "/home/all3n/viorb_config/config/ORBvoc.bin";
      //string orbVocFile = "/home/robooster/viorb_config/config/ORBvoc.txt";
      string orbSetiingsFile = "/home/all3n/viorb_config/config/kinect2_sd.yaml";

=======
      string orbVocFile = "/home/jinglun/viorb_config/config/ORBvoc.bin";
      //string orbVocFile = "/home/robooster/viorb_config/config/ORBvoc.txt";
      string orbSetiingsFile = "/home/jinglun/viorb_config/config/kinect2_sd.yaml";
>>>>>>> d44933e8356f6014b6c205e747bf5c9be8dd54d6
      #if 1
      ORB_SLAM2::Viewer* viewer;
      viewer = new ORB_SLAM2::PangolinViewer(orbSetiingsFile);
      orbslam = new ORB_SLAM2::System( orbVocFile, orbSetiingsFile ,ORB_SLAM2::System::RGBD, viewer );
      #endif
      
      #if 0
      ORB_SLAM2::ORBVocabulary voc;
      voc.loadFromBinaryFile("/home/robooster/viorb_config/config/ORBvoc.bin");
      ORB_SLAM2::Camera::Load(orbSetiingsFile);
      ORB_SLAM2::Map map;
      string mapename = "/home/robooster/poine_ws/src/poine_orbslam/map/mapdata.bin";
      map.Load(mapename, voc);
      if (0)
      {
      vector<ORB_SLAM2::KeyFrame*> vkf = map.GetAllKeyFrames();
      for (unsigned int i=0; i<vkf.size(); i++) {
        ORB_SLAM2::KeyFrame* kf = vkf[i];
        cerr << kf->mnId << " " << kf->mBowVec.size() << " " << kf->mFeatVec.size();
        //for (unsigned int j=0; j<kf->mBowVec.size(); j++)
        //	if ((j%51)==0) cerr << " " << kf->mBowVec[j];
        //for (unsigned int j=0; j<kf->mFeatVec.size(); j++)
        //	  if ((j%51)==0) cerr << " " << kf->mFeatVec[j];
        cerr << endl;
      }
      } 
      ORB_SLAM2::PangolinViewer viewer(orbSetiingsFile);
      
      
      orbslam = new ORB_SLAM2::System( orbVocFile, orbSetiingsFile, ORB_SLAM2::System::RGBD, &viewer, &map, &voc );
      viewer.RegisterMap(&map);
      viewer.Run();
      
      #endif
   }
  
   ~Receiver()
   {
       if (orbslam)
       {
           orbslam->Shutdown();
           //orbslam->shutdownandsave();
           delete orbslam;
       }
   }
<<<<<<< HEAD
   
=======
>>>>>>> d44933e8356f6014b6c205e747bf5c9be8dd54d6
 
   void run(const Mode mode)
   {
     start(mode);
     stop();
   }
 
   void finish() 
   {
   }
 private:
   void start(const Mode mode)
   {
     this->mode = mode;
     running = true;
 
     std::string topicCameraInfoColor = topicColor.substr(0, topicColor.rfind('/')) + "/camera_info";
     std::string topicCameraInfoDepth = topicDepth.substr(0, topicDepth.rfind('/')) + "/camera_info";
 
     image_transport::TransportHints hints(useCompressed ? "compressed" : "raw");
     subImageColor = new image_transport::SubscriberFilter(it, topicColor, queueSize, hints);
     subImageDepth = new image_transport::SubscriberFilter(it, topicDepth, queueSize, hints);
     subCameraInfoColor = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoColor, queueSize);
     subCameraInfoDepth = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoDepth, queueSize);
 
     if(useExact)
     {
       syncExact = new message_filters::Synchronizer<ExactSyncPolicy>(ExactSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
       syncExact->registerCallback(boost::bind(&Receiver::callback, this, _1, _2, _3, _4));
     }
     else
     {
       syncApproximate = new message_filters::Synchronizer<ApproximateSyncPolicy>(ApproximateSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
       syncApproximate->registerCallback(boost::bind(&Receiver::callback, this, _1, _2, _3, _4));
     }
 
     spinner.start();
 
     std::chrono::milliseconds duration(1);
     while(!updateImage || !updateCloud)
     {
       if(!ros::ok())
       {
         return;
       }
       std::this_thread::sleep_for(duration);
    }
     createLookup(this->color.cols, this->color.rows);
 
     switch(mode)
     {
     case IMAGE:
       imageViewer();
       break;
     case BOTH:
       imageViewerThread = std::thread(&Receiver::imageViewer, this);
       break;
     }
   }
 
   void stop()
   {
     spinner.stop();
 
     if(useExact)
     {
       delete syncExact;
     }
     else
     {
       delete syncApproximate;
     }
 
     delete subImageColor;
     delete subImageDepth;
     delete subCameraInfoColor;
     delete subCameraInfoDepth;
 
     running = false;
     if(mode == BOTH)
     {
       imageViewerThread.join();
     }
   }
 
   void callback(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth,
                 const sensor_msgs::CameraInfo::ConstPtr cameraInfoColor, const sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth)
   {
     cv::Mat color, depth;
 
     readCameraInfo(cameraInfoColor, cameraMatrixColor);
     readCameraInfo(cameraInfoDepth, cameraMatrixDepth);
     readImage(imageColor, color);
     readImage(imageDepth, depth);
 
     // IR image input
     if(color.type() == CV_16U)
     {
       cv::Mat tmp;
       color.convertTo(tmp, CV_8U, 0.02);
       cv::cvtColor(tmp, color, CV_GRAY2BGR);
     }
 
    lock.lock();
     this->color = color;
     this->depth = depth;
     updateImage = true;
     updateCloud = true;
     lock.unlock();
   }

<<<<<<< HEAD
   

=======
>>>>>>> d44933e8356f6014b6c205e747bf5c9be8dd54d6
   void imageViewer()
   {
     
     cv::Mat color, depth;
<<<<<<< HEAD
     char bck[100]="~/";
     char colorfile[10] = "/color";
     char depthfile[10] = "/depth";
     tat=getTime();
     char nyx[100];
     stringstream sta;
     sta<<tat;
     sta>>nyx;
     char bak1[100]="mkdir -p ";
     char bak2[100]="mkdir -p ";
     strcat(bak1,bck);
     strcat(bak1,nyx);
     strcat(bak1,colorfile);
     system(bak1); 
     strcat(bak2,bck);
     strcat(bak2,nyx);
     strcat(bak2,depthfile);
     system(bak2); 
=======
>>>>>>> d44933e8356f6014b6c205e747bf5c9be8dd54d6
     for(; running && ros::ok();)
     {
       if(updateImage)
       {
         lock.lock();
         color = this->color;
         depth = this->depth;
         updateImage = false;
         lock.unlock();
 
         if (orbslam)
         {

<<<<<<< HEAD
          orbslam->TrackRGBD( color, depth, ros::Time::now().toSec(),tat );
=======
          orbslam->TrackRGBD( color, depth, ros::Time::now().toSec() );
>>>>>>> d44933e8356f6014b6c205e747bf5c9be8dd54d6

         }

       }

     }
 
     cv::destroyAllWindows();
     cv::waitKey(100);
   }
 
   void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image) const
   {
     cv_bridge::CvImageConstPtr pCvImage;
     pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
     pCvImage->image.copyTo(image);
   }
 
   void readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo, cv::Mat &cameraMatrix) const
   {
     double *itC = cameraMatrix.ptr<double>(0, 0);
     for(size_t i = 0; i < 9; ++i, ++itC)
     {
       *itC = cameraInfo->K[i];
     }
   }
 
   void dispDepth(const cv::Mat &in, cv::Mat &out, const float maxValue)
   {
     cv::Mat tmp = cv::Mat(in.rows, in.cols, CV_8U);
     const uint32_t maxInt = 255;
 
     #pragma omp parallel for
     for(int r = 0; r < in.rows; ++r)
     {
       const uint16_t *itI = in.ptr<uint16_t>(r);
       uint8_t *itO = tmp.ptr<uint8_t>(r);
 
       for(int c = 0; c < in.cols; ++c, ++itI, ++itO)
       {
         *itO = (uint8_t)std::min((*itI * maxInt / maxValue), 255.0f);
       }
     }
 
     cv::applyColorMap(tmp, out, cv::COLORMAP_JET);
   }

   void combine(const cv::Mat &inC, const cv::Mat &inD, cv::Mat &out)
   {
     out = cv::Mat(inC.rows, inC.cols, CV_8UC3);
 
     #pragma omp parallel for
     for(int r = 0; r < inC.rows; ++r)
     {
       const cv::Vec3b
      *itC = inC.ptr<cv::Vec3b>(r),
        *itD = inD.ptr<cv::Vec3b>(r);
       cv::Vec3b *itO = out.ptr<cv::Vec3b>(r);
 
      for(int c = 0; c < inC.cols; ++c, ++itC, ++itD, ++itO)
       {
         itO->val[0] = (itC->val[0] + itD->val[0]) >> 1;
         itO->val[1] = (itC->val[1] + itD->val[1]) >> 1;
         itO->val[2] = (itC->val[2] + itD->val[2]) >> 1;
       }
     }
   }
 
 
   void createLookup(size_t width, size_t height)
  {
     const float fx = 1.0f / cameraMatrixColor.at<double>(0, 0);
     const float fy = 1.0f / cameraMatrixColor.at<double>(1, 1);
     const float cx = cameraMatrixColor.at<double>(0, 2);
     const float cy = cameraMatrixColor.at<double>(1, 2);
     float *it;
 
     lookupY = cv::Mat(1, height, CV_32F);
     it = lookupY.ptr<float>();
     for(size_t r = 0; r < height; ++r, ++it)
     {
       *it = (r - cy) * fy;
     }
 
     lookupX = cv::Mat(1, width, CV_32F);
     it = lookupX.ptr<float>();
     for(size_t c = 0; c < width; ++c, ++it)
     {
       *it = (c - cx) * fx;
     }
   }
 };



 int main(int argc, char **argv)
 {
 ros::init(argc, argv, "poine_orbslam", ros::init_options::AnonymousName);
   if(!ros::ok())
   {
     return 0;
   }
<<<<<<< HEAD
   
   ros::NodeHandle nh;
   ros::Subscriber sub;
   //sub = nh.subscribe<sensor_msgs::Imu>("/imu_os3dm/imu_raw", 50, &IMUCallback);
   //sub = nh.subscribe<imu_os3dm::imu>("/imu_os3dm/imu_orb", 50, &IMUCallback);
   //sub = nh.subscribe<nav_msgs::Odometry>("/mobile_base_controller/odom", 50, &BaseCallback);
   //ofstream outfile("//home//robooster//Desktop//Odometry_data.txt");
   //outfile<<"world_time"<<"\t\t  "<<"left"<<"\t\t\t\t"<<"front"<<"\t\t\t\t"<<"above"<<endl;
   //ofstream outfile1("//home//robooster//Desktop//orbslam_data.txt");
   //outfile1<<"world_time"<<"\t\t  "<<"q_x"<<"\t\t\t\t"<<"q_y"<<"\t\t\t\t"<<"q_z"<<"\t\t\t\t"<<"q_w"<<"\t\t\t\t"<<"t_x"<<"\t\t\t\t"<<"t_y"<<"\t\t\t\t"<<"t_z"<<"\t\t\t\t"<<endl;
   //ofstream outfile2("//home//robooster//Desktop//imu_data_raw.txt");
   //ofstream outfile2("//home//robooster//Desktop//imu_data_orb.txt");  
   //outfile2<<"world_time"<<"\t\t  "<<"q_x"<<"\t\t\t\t"<<"q_y"<<"\t\t\t\t"<<"q_z"<<"\t\t\t\t"<<"q_w"<<endl;
   //outfile2<<"world_time"<<"\t\t  "<<"acc_x"<<"\t\t\t "<<"acc_y"<<"\t\t\t "<<"acc_z"<<"\t\t\t ";
   //outfile2<<"mag_x"<<"\t\t\t "<<"mag_y"<<"\t\t\t "<<"mag_z"<<"\t\t\t ";
   //outfile2<<"gyro_x"<<"\t\t\t "<<"gyro_y"<<"\t\t\t "<<"gyro_z"<<"\t\t\t "<<endl;
   std::string topicColor = "/kinect2/qhd/image_color_rect";
   std::string topicDepth = "/kinect2/qhd/image_depth_rect";
=======
   std::string topicColor = "/kinect2/sd/image_color_rect";
   std::string topicDepth = "/kinect2/sd/image_depth_rect";
>>>>>>> d44933e8356f6014b6c205e747bf5c9be8dd54d6
   bool useExact = true;
   bool useCompressed = false;
   Receiver::Mode mode = Receiver::IMAGE;
   // 初始化receiver
   Receiver receiver(topicColor, topicDepth, useExact, useCompressed);
 
   //OUT_INFO("starting receiver...");
   receiver.run(mode);
 
   receiver.finish();

<<<<<<< HEAD
   

=======
>>>>>>> d44933e8356f6014b6c205e747bf5c9be8dd54d6
   ros::shutdown();
 
return 0;

}
