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
  
  #include <message_filters/subscriber.h>
  #include <message_filters/synchronizer.h>
  #include <message_filters/sync_policies/exact_time.h>
  #include <message_filters/sync_policies/approximate_time.h>
  
  #include <kinect2_bridge/kinect2_definitions.h>

  #include "System.h"
  #include "pointcloudmapping.h"
  #include "PangolinViewer.h"
  #include "Viewer.h"
  #include <signal.h>  
  #include <geometry_msgs/Twist.h> 
  ros::Publisher cmdVelPub;

  void shutdown(int sig)  
  {  
    cmdVelPub.publish(geometry_msgs::Twist());//使机器人停止运动  
    ROS_INFO("move_turtle_goforward ended!");  
    ros::shutdown();  
  } 

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
      string orbVocFile = "/home/robooster/viorb_config/config/ORBvoc.bin";
      //string orbVocFile = "/home/robooster/viorb_config/config/ORBvoc.txt";
      string orbSetiingsFile = "/home/robooster/viorb_config/config/kinect2_sd.yaml";
>>>>>>> d44933e8356f6014b6c205e747bf5c9be8dd54d6
      #if 0
      ORB_SLAM2::Viewer* viewer;
      viewer = new ORB_SLAM2::PangolinViewer(orbSetiingsFile);
      orbslam = new ORB_SLAM2::System( orbVocFile, orbSetiingsFile ,ORB_SLAM2::System::RGBD, viewer );
      #endif
      
      #if 1
      ORB_SLAM2::ORBVocabulary voc;
<<<<<<< HEAD
      voc.loadFromBinaryFile("/home/all3n/viorb_config/config/ORBvoc.bin");
      ORB_SLAM2::Camera::Load(orbSetiingsFile);
      ORB_SLAM2::Map map;
      string mapename = "/home/all3n/viorb_config/save_map/mapdata.bin";
=======
      voc.loadFromBinaryFile("/home/robooster/viorb_config/config/ORBvoc.bin");
      ORB_SLAM2::Camera::Load(orbSetiingsFile);
      ORB_SLAM2::Map map;
      string mapename = "/home/robooster/viorb_config/save_map/mapdata.bin";
>>>>>>> d44933e8356f6014b6c205e747bf5c9be8dd54d6
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
      
      
      //orbslam = new ORB_SLAM2::System( orbVocFile, orbSetiingsFile, ORB_SLAM2::System::RGBD, &viewer, &map, &voc );
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

   void imageViewer()
   {
     
     cv::Mat color, depth;
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

          orbslam->TrackRGBD( color, depth, ros::Time::now().toSec() );

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

   ros::shutdown();
 
return 0;

}
