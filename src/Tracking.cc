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


#include "Tracking.h"

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include "poine_orbslam/Matrix.h"
#include <boost/foreach.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "ORBmatcher.h"
#include "Converter.h"
#include "Camera.h"
#include "Map.h"
#include "Initializer.h"

#include "Optimizer.h"
#include "PnPsolver.h"
#include "pointcloudmapping.h"
#include <iostream>
#include <fstream>

#include <mutex>


using namespace std;
#if 0
void GrabRGBD_IMU(const poine_orbslam::MatrixConstPtr orb_msg, const sensor_msgs::ImuConstPtr& imu_msg, const nav_msgs::OdometryConstPtr& odom_msg)
{
    //bag_record.write("/kinect2/qhd/image_color_rect", kinect2color_msg->header.stamp.now(), kinect2color_msg);
    //bag_record.write("/kinect2/qhd/image_depth_rect", kinect2depth_msg->header.stamp.now(), kinect2depth_msg);
    //bag_record.write("/imu_os3dm/imu_raw", imu_msg->header.stamp.now(), imu_msg);

            cout<<"can we get here?"<<endl;
    
    // IMU 数据保存
    ofstream outfile2("//home//robooster//Desktop//imu_data_raw.txt", ios_base::app);
    Eigen::Quaterniond q;
    q.x() = imu_msg->orientation.x;
    q.y() = imu_msg->orientation.y;
    q.z() = imu_msg->orientation.z;
    q.w() = imu_msg->orientation.w;
    outfile2.setf(ios::fixed);
    outfile2<< fixed << setprecision(5) << std::left << std::setw(18)<<imu_msg->header.stamp.now().toSec();
    outfile2<< fixed << setprecision(8) << std::left << std::setw(15)<<q.x();
    outfile2<< fixed << setprecision(8) << std::left << std::setw(15)<<q.y();
    outfile2<< fixed << setprecision(8) << std::left << std::setw(15)<<q.z();
    outfile2<< fixed << setprecision(8) << std::left << std::setw(15)<<q.w()<<endl;

    // Odom 数据保存
    static double px, py, pz;               
    pz = odom_msg->pose.pose.position.x;
    px = odom_msg->pose.pose.position.y;
    py = odom_msg->pose.pose.position.z;
    ofstream outfile("//home//robooster//Desktop//odometry_data.txt", ios_base::app);
    outfile.setf(ios::fixed);
    outfile<< fixed << setprecision(5) << std::left << std::setw(18)<<odom_msg->header.stamp.now().toSec();
    outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<px;
    outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<pz;
    outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<py<<endl;

    #if 1
    ofstream outfile1("//home//robooster//Desktop//orbslam_data.txt", ios_base::app);
    outfile1.setf(ios::fixed);
    outfile1<< fixed << setprecision(5) << std::left << std::setw(18)<<orb_msg->header.stamp.now().toSec();
    outfile1<< fixed << setprecision(8) << std::left << std::setw(15)<<orb_msg->Frame_mTcw.rotation.x;
    outfile1<< fixed << setprecision(8) << std::left << std::setw(15)<<orb_msg->Frame_mTcw.rotation.y;
    outfile1<< fixed << setprecision(8) << std::left << std::setw(15)<<orb_msg->Frame_mTcw.rotation.z;
    outfile1<< fixed << setprecision(8) << std::left << std::setw(15)<<orb_msg->Frame_mTcw.rotation.w;
    outfile1<< fixed << setprecision(8) << std::left << std::setw(15)<<orb_msg->Frame_mTcw.translation.x;
    outfile1<< fixed << setprecision(8) << std::left << std::setw(15)<<orb_msg->Frame_mTcw.translation.z;
    outfile1<< fixed << setprecision(8) << std::left << std::setw(15)<<orb_msg->Frame_mTcw.translation.y<<endl;
    #endif
}
#endif
namespace ORB_SLAM2
{

Tracking::Tracking(System *pSys, ORBVocabulary* pVoc, Map *pMap, shared_ptr<PointCloudMapping> pPointCloud, KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor):
   mSensor(sensor), mbOnlyTracking(false), mbVO(false), mpORBVocabulary(pVoc), mpPointCloudMapping( pPointCloud ), mpKeyFrameDB(pKFDB), mpInitializer(static_cast<Initializer*>(NULL)), mpSystem(pSys), mpMap(pMap), mnLastRelocFrameId(0)
{
  if (pMap->KeyFramesInMap() == 0)
	mState = NO_IMAGES_YET;
  else {
	mState = LOST;
	mbOnlyTracking = true;
	std::vector<KeyFrame*> akf = pMap->GetAllKeyFrames();
	mpReferenceKF = akf[0];
  }

    // Load camera parameters from settings file

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    
    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    cout << "- fps: " << fps << endl;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = fps;

    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    // Load ORB parameters

    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==System::STEREO)
        mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==System::MONOCULAR)
        mpIniORBextractor = new ORBextractor(2*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    cout << endl  << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

    if(sensor==System::STEREO || sensor==System::RGBD)
    {
      //mThDepth = mbf*(float)fSettings["ThDepth"]/fx;
      mThDepth = Camera::bf*(float)fSettings["ThDepth"]/Camera::K.at<float>(0,0);
        cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
    }

    if(sensor==System::RGBD)
    {
        mDepthMapFactor = fSettings["DepthMapFactor"];
        if(mDepthMapFactor==0)
            mDepthMapFactor=1;
        else
            mDepthMapFactor = 1.0f/mDepthMapFactor;
    }
    fSettings.release();
}

void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
{
    mpLoopClosing=pLoopClosing;
}

void Tracking::SetViewer(Viewer *pViewer)
{
    mpViewer=pViewer;
}


cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp)
{
    mImGray = imRectLeft;
    cv::Mat imGrayRight = imRectRight;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGB2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGR2GRAY);
        }
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGBA2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGRA2GRAY);
        }
    }

    mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mThDepth);

    Track();

    return mCurrentFrame.mTcw.clone();
}

cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp,string tat)
{
    //mImGray = imRGB;
    //cv::Mat imDepth = imD;
    mImRGB = imRGB;
    mImGray = imRGB;
    mImDepth = imD;
    // stringstream sto;
    // string sda;
    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    if(mDepthMapFactor!=1 || mImDepth.type()!=CV_32F);
    mImDepth.convertTo(mImDepth,CV_32F,mDepthMapFactor);

    mCurrentFrame = Frame(mImGray,mImDepth,timestamp,mpORBextractorLeft,mpORBVocabulary,mThDepth);
    // int x=mCurrentFrame.mnId;
    // cout<<x<<endl;
    Track(tat);
    return mCurrentFrame.mTcw.clone();
}

cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp)
{
    //mImGray = imRGB;
    //cv::Mat imDepth = imD;
    mImRGB = imRGB;
    mImGray = imRGB;
    mImDepth = imD;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    if(mDepthMapFactor!=1 || mImDepth.type()!=CV_32F);
    mImDepth.convertTo(mImDepth,CV_32F,mDepthMapFactor);

    mCurrentFrame = Frame(mImGray,mImDepth,timestamp,mpORBextractorLeft,mpORBVocabulary,mThDepth);

    Track();

    return mCurrentFrame.mTcw.clone();
}

cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp, const Eigen::Quaterniond &q, const Eigen::Matrix<double,3,1> &imu_a, const Eigen::Matrix<double,3,1> &imu_w, const Eigen::Matrix<double,3,1> &odom)
{
    //mImGray = imRGB;
    //cv::Mat imDepth = imD;
    mImRGB = imRGB;
    mImGray = imRGB;
    mImDepth = imD;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    if(mDepthMapFactor!=1 || mImDepth.type()!=CV_32F);
    mImDepth.convertTo(mImDepth,CV_32F,mDepthMapFactor);

    mCurrentFrame = Frame(mImGray,mImDepth,timestamp,mpORBextractorLeft,mpORBVocabulary,mThDepth);

    Track(q,imu_a,imu_w,odom);

    return mCurrentFrame.mTcw.clone();
}


cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp)
{
    mImGray = im;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)
        mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mThDepth);
    else
        mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mThDepth);

    Track();

    return mCurrentFrame.mTcw.clone();
}


bool Tracking::_Track_full() {
  bool bOK;
  if(mState==OK)
	{
	  // Local Mapping might have changed some MapPoints tracked in last frame
	  CheckReplacedInLastFrame();
	  
	  if(mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2)
		{
		  bOK = TrackReferenceKeyFrame();
		}
	  else
		{
		  bOK = TrackWithMotionModel();
		  if(!bOK)
			bOK = TrackReferenceKeyFrame();
		}
	}
  else
	{
	  bOK = Relocalization();
	}
  return bOK;
}

bool Tracking::_Track_loc_only() {
  bool bOK;
  // Only Tracking: Local Mapping is deactivated
  //cerr << " Tracking: initialized and  mbOnlyTracking " << endl;
  if(mState==LOST)
	{
	  bOK = Relocalization();
	  //cerr << " Lost, doing relocatization " << bOK << endl;
	}
  else
	{
	  if(!mbVO)
		{
		  // In last frame we tracked enough MapPoints in the map
		  // cerr << "we tracked enough map points" << endl;
		  if(!mVelocity.empty())
			{
			  bOK = TrackWithMotionModel();
			}
		  else
			{
			  cerr << "vel empty" << endl;
			  bOK = TrackReferenceKeyFrame();
			}
		}
	  else
		{
		  // In last frame we tracked mainly "visual odometry" points.
		  //cerr << "we tracked mostly odometry" << endl;
		  // We compute two camera poses, one from motion model and one doing relocalization.
		  // If relocalization is sucessfull we choose that solution, otherwise we retain
		  // the "visual odometry" solution.
		  
		  bool bOKMM = false;
		  bool bOKReloc = false;
		  vector<MapPoint*> vpMPsMM;
		  vector<bool> vbOutMM;
		  cv::Mat TcwMM;
		  if(!mVelocity.empty())
			{
			  bOKMM = TrackWithMotionModel();
			  vpMPsMM = mCurrentFrame.mvpMapPoints;
			  vbOutMM = mCurrentFrame.mvbOutlier;
			  TcwMM = mCurrentFrame.mTcw.clone();
			}
		  bOKReloc = Relocalization();
		  
		  if(bOKMM && !bOKReloc)
			{
			  mCurrentFrame.SetPose(TcwMM);
			  mCurrentFrame.mvpMapPoints = vpMPsMM;
			  mCurrentFrame.mvbOutlier = vbOutMM;
			  
			  if(mbVO)
				{
				  for(int i =0; i<mCurrentFrame.N; i++)
					{
					  if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
						{
						  mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
						}
					}
				}
			}
		  else if(bOKReloc)
			{
			  mbVO = false;
			}
		  
		  bOK = bOKReloc || bOKMM;
		}
	}
  return bOK;
}

#if 0
void Tracking::BaseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // 相机的x轴是正左(X方向)，z轴是正前(Y方向)
    // 小车的y轴是正左(X方向)，x轴是正前(Y方向)
    pz = msg->pose.pose.position.x;
    px = msg->pose.pose.position.y;
    py = msg->pose.pose.position.z;
    ofstream outfile("//home//robooster//Desktop//odometry_data.txt", ios_base::app);
    outfile.setf(ios::fixed);
    outfile<< fixed << setprecision(5) << std::left << std::setw(18)<<msg->header.stamp.now().toSec();
    outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<px;
    outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<pz;
    outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<py<<endl;    
    //cout<<"px = "<<px<<endl;
}
#endif

#if 0
void Tracking::ImuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    cout<<"test!!!!!!!!!"<<endl;
    ofstream outfile("//home//robooster//Desktop//imu_data_for_orb.txt", ios_base::app);
    outfile.setf(ios::fixed);
    Eigen::Quaterniond q;
    q.x() = msg->orientation.x;
    q.y() = msg->orientation.y;
    q.z() = msg->orientation.z;
    q.w() = msg->orientation.w;
    outfile<< fixed << setprecision(5) << std::left << std::setw(18)<<msg->header.stamp.now().toSec();
    outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<q.x();
    outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<q.y();
    outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<q.z();
    outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<q.w()<<endl;
}
#endif

void Tracking::Track()
{
    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState=mState;

    // Get Map Mutex -> Map cannot be changed
    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

    if(mState==NOT_INITIALIZED)
    {
        if(mSensor==System::STEREO || mSensor==System::RGBD)
            StereoInitialization();
        else
            MonocularInitialization();

		if (mpViewer != NULL) mpViewer->UpdateFrame(this);

        if(mState!=OK)
            return;
    }
    else
    {
        // System is initialized. Track Frame.
        bool bOK;

        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        if(!mbOnlyTracking)
        {
		    // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.
		  bOK = _Track_full();
        }
        else
        {
		  bOK = _Track_loc_only();
		}
        mCurrentFrame.mpReferenceKF = mpReferenceKF;

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        if(!mbOnlyTracking)
        {
            if(bOK)
                bOK = TrackLocalMap();
        }
        else
        {
            // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
            // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
            // the camera we will use the local map again.
            if(bOK && !mbVO)
                bOK = TrackLocalMap();
        }

        if(bOK)
            mState = OK;
        else
            mState=LOST;

        // Update drawer
        if (mpViewer != NULL) mpViewer->UpdateFrame(this);

        // If tracking were good, check if we insert a keyframe
        if(bOK)
        {
            // Update motion model
            if(!mLastFrame.mTcw.empty())
            {
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
                mVelocity = mCurrentFrame.mTcw*LastTwc;
            }
            else
                mVelocity = cv::Mat();

			if (mpViewer != NULL) mpViewer->SetCurrentCameraPose(mCurrentFrame.mTcw);

	    //cerr << mCurrentFrame.mTcw.rowRange(0,3).col(3) << endl;

            // Clean temporal point matches
            for(int i=0; i<mCurrentFrame.N; i++)
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(pMP)
                    if(pMP->Observations()<1)
                    {
                        mCurrentFrame.mvbOutlier[i] = false;
                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    }
            }

            // Delete temporal MapPoints
            for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
            {
                MapPoint* pMP = *lit;
                delete pMP;
            }
            mlpTemporalPoints.clear();
            
            #if 0
            //odom_sub = nh.subscribe<nav_msgs::Odometry>("/mobile_base_controller/odom", 50, &Tracking::BaseCallback, this);
            //mCurrentFrame.mTcw.at<float>(0,3) = static_cast<float>(px);
            //mCurrentFrame.mTcw.at<float>(1,3) = static_cast<float>(py);
            //mCurrentFrame.mTcw.at<float>(2,3) = static_cast<float>(pz);
            //imu_sub = nh.subscribe<sensor_msgs::Imu>("/IMU_for_orb", 1000, &Tracking::ImuCallback, this);
            #endif

            #if 0
            //cout<<"mCurrentFrame.mnId = "<<mCurrentFrame.mnId<<endl;
            Frame_pub = nh.advertise<poine_orbslam::Matrix>("Frame_Tcw", 50);
            Frame_Tcw.header.stamp = ros::Time::now();
            Frame_Tcw.header.seq = mCurrentFrame.mnId;
            //Frame_Tcw.header.frame_id = "imu_link";
            mCurrentFrame.UpdatePoseMatrices();
            cv::Mat t = mCurrentFrame.mOw;
            Frame_Tcw.Frame_mTcw.translation.x = t.at<float>(0);
            Frame_Tcw.Frame_mTcw.translation.y = t.at<float>(1);
            Frame_Tcw.Frame_mTcw.translation.z = t.at<float>(2);
            cv::Mat R = mCurrentFrame.mTcw.rowRange(0,3).colRange(0,3).t();
            vector<float> v = Converter::toQuaternion(R);
            //vector<float> v = Converter::toQuaternion(mCurrentFrame.mTcw);
            Frame_Tcw.Frame_mTcw.rotation.x = v[0];
            Frame_Tcw.Frame_mTcw.rotation.y = v[1];
            Frame_Tcw.Frame_mTcw.rotation.z = v[2];
            Frame_Tcw.Frame_mTcw.rotation.w = v[3];
            Frame_pub.publish(Frame_Tcw);
            #endif

            #if 0
            imu_pub = nh.advertise<sensor_msgs::Imu>("Imu_Frame", 50);
            imu_Frame.header.frame_id = "imu_link";
            imu_Frame.orientation.x = v[0];
            imu_Frame.orientation.y = v[1];
            imu_Frame.orientation.z = v[2];
            imu_Frame.orientation.w = v[3];
            imu_pub.publish(imu_Frame);
            #endif

            #if 0
            ofstream outfile("//home//robooster//Desktop//orbslam_data.txt", ios_base::app);
            outfile.setf(ios::fixed);
            outfile<< fixed << setprecision(5) << std::left << std::setw(18)<<Frame_Tcw.header.stamp.now().toSec();
            outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<v[0];
            outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<v[1];
            outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<v[2];
            outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<v[3];
            outfile<< fixed << setprecision(8) << std::left << std::setw(20)<<t.at<float>(0);
            outfile<< fixed << setprecision(8) << std::left << std::setw(20)<<t.at<float>(2);
            outfile<< fixed << setprecision(8) << std::left << std::setw(20)<<t.at<float>(1)<<endl;
            #endif



            // Check if we need to insert a new keyframe
            if(NeedNewKeyFrame())
                CreateNewKeyFrame();

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame.
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            }
        }

        // Reset if the camera get lost soon after initialization
        if(mState==LOST)
        {
            if(mpMap->KeyFramesInMap()<=5)
            {
                cout << "Track lost soon after initialisation, reseting..." << endl;
                mpSystem->Reset();
                return;
            }
        }

        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        mLastFrame = Frame(mCurrentFrame);
    }

    // Store frame pose information to retrieve the complete camera trajectory afterwards.
    if(!mCurrentFrame.mTcw.empty())
    {
	    //cerr << "in tracking: !mCurrentFrame.mTcw.empty(), state " << mState << endl;
        cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();
        mlRelativeFramePoses.push_back(Tcr);
        mlpReferences.push_back(mpReferenceKF);
        mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
        mlbLost.push_back(mState==LOST);
    }
    else
    {
        // This can happen if tracking is lost
        mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
        mlpReferences.push_back(mlpReferences.back());
        mlFrameTimes.push_back(mlFrameTimes.back());
        mlbLost.push_back(mState==LOST);
    }

}

void Tracking::Track(const Eigen::Quaterniond &q, const Eigen::Matrix<double,3,1> &imu_a, const Eigen::Matrix<double,3,1> &imu_w, const Eigen::Matrix<double,3,1> &odom)
{
    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState=mState;

    // Get Map Mutex -> Map cannot be changed
    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

    if(mState==NOT_INITIALIZED)
    {
        if(mSensor==System::STEREO || mSensor==System::RGBD)
            StereoInitialization();
        else
            MonocularInitialization();

		if (mpViewer != NULL) mpViewer->UpdateFrame(this);

        if(mState!=OK)
            return;
    }
    else
    {
        // System is initialized. Track Frame.
        bool bOK;

        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        if(!mbOnlyTracking)
        {
		    // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.
		  bOK = _Track_full();
        }
        else
        {
		  bOK = _Track_loc_only();
		}
        mCurrentFrame.mpReferenceKF = mpReferenceKF;

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        if(!mbOnlyTracking)
        {
            if(bOK)
                bOK = TrackLocalMap();
        }
        else
        {
            // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
            // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
            // the camera we will use the local map again.
            if(bOK && !mbVO)
                bOK = TrackLocalMap();
        }

        if(bOK)
            mState = OK;
        else
            mState=LOST;

        // Update drawer
        if (mpViewer != NULL) mpViewer->UpdateFrame(this);

        // If tracking were good, check if we insert a keyframe
        if(bOK)
        {
            // Update motion model
            if(!mLastFrame.mTcw.empty())
            {
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
                mVelocity = mCurrentFrame.mTcw*LastTwc;
            }
            else
                mVelocity = cv::Mat();

			if (mpViewer != NULL) mpViewer->SetCurrentCameraPose(mCurrentFrame.mTcw);

	    //cerr << mCurrentFrame.mTcw.rowRange(0,3).col(3) << endl;

            // Clean temporal point matches
            for(int i=0; i<mCurrentFrame.N; i++)
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(pMP)
                    if(pMP->Observations()<1)
                    {
                        mCurrentFrame.mvbOutlier[i] = false;
                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    }
            }

            // Delete temporal MapPoints
            for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
            {
                MapPoint* pMP = *lit;
                delete pMP;
            }
            mlpTemporalPoints.clear();
            
            //cout<<"mCurrentFrame.mnId = "<<mCurrentFrame.mnId<<endl;
            //Frame_Tcw.header.frame_id = "imu_link";

            /**
            * @brief: 将ORB坐标系转换到imu体坐标系（因此imupose中未再次做坐标变换）
            * @param: R:世界坐标系下，相机的旋转矩阵
            *         r:R的旋转向量
            *         v/q_:旋转矩阵R到四元数的变换
            *         eulr_orb_qua:相机旋转欧拉角
            *         t:世界坐标系下，相机的平移矩阵 
            **/
            mCurrentFrame.UpdatePoseMatrices();
            cv::Mat ttt = mCurrentFrame.mtcw;
            //cout<<ttt.at<float>(0)<<" "<<ttt.at<float>(1)<<" "<<ttt.at<float>(2)<<endl;
            cv::Mat R = mCurrentFrame.mRwc;
            cv::Mat r;
            cv::Rodrigues(R, r);
            swap(r.at<float>(1),r.at<float>(2));
            r.at<float>(0) = -r.at<float>(0);
            r.at<float>(1) = -r.at<float>(1);
            r.at<float>(2) = -r.at<float>(2);
            cv::Rodrigues(r, R);
            vector<float> v = Converter::toQuaternion(R);
            Eigen::Quaterniond q_;
            q_.x() = v[0];
            q_.y() = v[1];
            q_.z() = v[2];
            q_.w() = v[3];
            vector<float> eulr_orb_qua = Converter::toEulr(q_);
            //cv::Mat t = -R * mCurrentFrame.mtcw;
            cv::Mat t = mCurrentFrame.mOw;
            Eigen::Matrix<double,3,1> pose;
            pose << t.at<float>(0), t.at<float>(1), t.at<float>(2);

            #if 1
            ofstream outfile("//home//robooster//Desktop//orbslam_data.txt", ios_base::app);
            outfile.setf(ios::fixed);
            outfile<< fixed << setprecision(5) << std::left << std::setw(18)<<Frame_Tcw.header.stamp.now().toSec();
            #if 1
            outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<v[0];
            outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<v[1];
            outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<v[2];
            outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<v[3];
            #endif
            outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<eulr_orb_qua[0]*180/3.1415926;
            outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<eulr_orb_qua[1]*180/3.1415926;
            outfile<< fixed << setprecision(8) << std::left << std::setw(15)<<eulr_orb_qua[2]*180/3.1415926;
            outfile<< fixed << setprecision(8) << std::left << std::setw(20)<<t.at<float>(0);
            outfile<< fixed << setprecision(8) << std::left << std::setw(20)<<t.at<float>(1);
            outfile<< fixed << setprecision(8) << std::left << std::setw(20)<<t.at<float>(2)<<endl;
            #endif

            /**
            * @brief: 
            * @param: eulr_imu:imu姿态欧拉角
            *         q:主函数里传过来的imu姿态四元数
            **/
            vector<float> eulr_imu = Converter::toEulr(q);
            ofstream outfile2("//home//robooster//Desktop//imu_data_for_orb.txt", ios_base::app);
            outfile2.setf(ios::fixed);
            outfile2<< fixed << setprecision(5) << std::left << std::setw(18)<<ros::Time::now().toSec();
            #if 0
            outfile2<< fixed << setprecision(8) << std::left << std::setw(15)<<q.x();
            outfile2<< fixed << setprecision(8) << std::left << std::setw(15)<<q.y();
            outfile2<< fixed << setprecision(8) << std::left << std::setw(15)<<q.z();
            outfile2<< fixed << setprecision(8) << std::left << std::setw(15)<<q.w()<<endl;
            #endif
            outfile2<< fixed << setprecision(8) << std::left << std::setw(15)<<eulr_imu[0]*180/3.1415926;
            outfile2<< fixed << setprecision(8) << std::left << std::setw(15)<<eulr_imu[1]*180/3.1415926;
            outfile2<< fixed << setprecision(8) << std::left << std::setw(15)<<eulr_imu[2]*180/3.1415926<<endl;

            ORB_SLAM2::Imuposeinf imu;
            // q:主函数里传过来的imu姿态四元数；imu_a:主函数里传过来的imu加计原始数据；imu_w:主函数里传过来的imu陀螺原始数据;
            imu.imuCallback(mCurrentFrame.mTimeStamp, q, imu_w); 
            imu.orbCallback(mCurrentFrame.mTimeStamp, q_);
            // q_fusion = imu.orbCallback(mCurrentFrame.mTimeStamp, q_);
            // q_fusion.x() = q.x();
            // q_fusion.y() = q.y();
            // q_fusion.w() = sqrt(1 - q_fusion.x()*q_fusion.x() - q_fusion.y()*q_fusion.y() - q_fusion.z()*q_fusion.z());
            // q_fusion = q;

            // Eigen::Matrix<double,3,3> _m = q_fusion.toRotationMatrix();
            // R_fusion = Converter::toCvMat(_m);
            // t_fusion = Converter::toCvMat(odom);
            // T_fusion = mCurrentFrame.mTcw;
            // // T_fusion = cv::Mat::eye(cv::Size(4, 4), CV_32F);
            // T_fusion.rowRange(0,3).colRange(0,3) = R_fusion.t();
            // // T_fusion.rowRange(0,3).col(3) = -R_fusion.inv() * t_fusion;
            // mCurrentFrame.SetPose(T_fusion);

            // Check if we need to insert a new keyframe
            if(NeedNewKeyFrame())
                CreateNewKeyFrame();

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame.
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            }
        }

        // Reset if the camera get lost soon after initialization
        if(mState==LOST)
        {
            if(mpMap->KeyFramesInMap()<=5)
            {
                cout << "Track lost soon after initialisation, reseting..." << endl;
                mpSystem->Reset();
                return;
            }
        }

        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        mLastFrame = Frame(mCurrentFrame);
    }

    // Store frame pose information to retrieve the complete camera trajectory afterwards.
    if(!mCurrentFrame.mTcw.empty())
    {
	    //cerr << "in tracking: !mCurrentFrame.mTcw.empty(), state " << mState << endl;
        cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();
        mlRelativeFramePoses.push_back(Tcr);
        mlpReferences.push_back(mpReferenceKF);
        mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
        mlbLost.push_back(mState==LOST);
    }
    else
    {
        // This can happen if tracking is lost
        mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
        mlpReferences.push_back(mlpReferences.back());
        mlFrameTimes.push_back(mlFrameTimes.back());
        mlbLost.push_back(mState==LOST);
    }

}

void Tracking::Track(string tat)
{
    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState=mState;

    // Get Map Mutex -> Map cannot be changed
    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

    if(mState==NOT_INITIALIZED)
    {
        if(mSensor==System::STEREO || mSensor==System::RGBD)
            StereoInitialization();
        else
            MonocularInitialization();

		if (mpViewer != NULL) mpViewer->UpdateFrame(this);

        if(mState!=OK)
            return;
    }
    else
    {
        // System is initialized. Track Frame.
        bool bOK;

        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        if(!mbOnlyTracking)
        {
		    // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.
		  bOK = _Track_full();
        }
        else
        {
		  bOK = _Track_loc_only();
		}
        mCurrentFrame.mpReferenceKF = mpReferenceKF;

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        if(!mbOnlyTracking)
        {
            if(bOK)
                bOK = TrackLocalMap();
        }
        else
        {
            // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
            // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
            // the camera we will use the local map again.
            if(bOK && !mbVO)
                bOK = TrackLocalMap();
        }

        if(bOK)
            mState = OK;
        else
            mState=LOST;

        // Update drawer
        if (mpViewer != NULL) mpViewer->UpdateFrame(this);

        // If tracking were good, check if we insert a keyframe
        if(bOK)
        {
            // Update motion model
            if(!mLastFrame.mTcw.empty())
            {
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
                mVelocity = mCurrentFrame.mTcw*LastTwc;
            }
            else
                mVelocity = cv::Mat();

			if (mpViewer != NULL) mpViewer->SetCurrentCameraPose(mCurrentFrame.mTcw);

	    //cerr << mCurrentFrame.mTcw.rowRange(0,3).col(3) << endl;

            // Clean temporal point matches
            for(int i=0; i<mCurrentFrame.N; i++)
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(pMP)
                    if(pMP->Observations()<1)
                    {
                        mCurrentFrame.mvbOutlier[i] = false;
                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    }
            }

            // Delete temporal MapPoints
            for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
            {
                MapPoint* pMP = *lit;
                delete pMP;
            }
            mlpTemporalPoints.clear();

            // Check if we need to insert a new keyframe
            if(NeedNewKeyFrame())
                CreateNewKeyFrame(tat);

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame.
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            }
        }

        // Reset if the camera get lost soon after initialization
        if(mState==LOST)
        {
            if(mpMap->KeyFramesInMap()<=5)
            {
                cout << "Track lost soon after initialisation, reseting..." << endl;
                mpSystem->Reset();
                return;
            }
        }

        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        mLastFrame = Frame(mCurrentFrame);
    }

    // Store frame pose information to retrieve the complete camera trajectory afterwards.
    if(!mCurrentFrame.mTcw.empty())
    {
	    //cerr << "in tracking: !mCurrentFrame.mTcw.empty(), state " << mState << endl;
        cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();
        mlRelativeFramePoses.push_back(Tcr);
        mlpReferences.push_back(mpReferenceKF);
        mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
        mlbLost.push_back(mState==LOST);
    }
    else
    {
        // This can happen if tracking is lost
        mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
        mlpReferences.push_back(mlpReferences.back());
        mlFrameTimes.push_back(mlFrameTimes.back());
        mlbLost.push_back(mState==LOST);
    }

}

void Tracking::StereoInitialization()
{
    if(mCurrentFrame.N>500)
    {
        // Set Frame pose to the origin
        mCurrentFrame.SetPose(cv::Mat::eye(4,4,CV_32F));

        // Create KeyFrame
        KeyFrame* pKFini = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

        // Insert KeyFrame in the map
        mpMap->AddKeyFrame(pKFini);

        // Create MapPoints and asscoiate to KeyFrame
        for(int i=0; i<mCurrentFrame.N;i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                MapPoint* pNewMP = new MapPoint(x3D,pKFini,mpMap);
                pNewMP->AddObservation(pKFini,i);
                pKFini->AddMapPoint(pNewMP,i);
                pNewMP->ComputeDistinctiveDescriptors();
                pNewMP->UpdateNormalAndDepth();
                mpMap->AddMapPoint(pNewMP);

                mCurrentFrame.mvpMapPoints[i]=pNewMP;
            }
        }

        cout << "New map created with " << mpMap->MapPointsInMap() << " points" << endl;

        mpLocalMapper->InsertKeyFrame(pKFini);

        mLastFrame = Frame(mCurrentFrame);
        mnLastKeyFrameId=mCurrentFrame.mnId;
        mpLastKeyFrame = pKFini;

        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints=mpMap->GetAllMapPoints();
        mpReferenceKF = pKFini;
        mCurrentFrame.mpReferenceKF = pKFini;

        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

        mpMap->mvpKeyFrameOrigins.push_back(pKFini);

		if (mpViewer != NULL) mpViewer->SetCurrentCameraPose(mCurrentFrame.mTcw);

        mState=OK;
    }
}

void Tracking::MonocularInitialization()
{

    if(!mpInitializer)
    {
        // Set Reference Frame
        if(mCurrentFrame.mvKeys.size()>100)
        {
            mInitialFrame = Frame(mCurrentFrame);
            mLastFrame = Frame(mCurrentFrame);
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
                mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;

            if(mpInitializer)
                delete mpInitializer;

            mpInitializer =  new Initializer(mCurrentFrame,1.0,200);

            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

            return;
        }
    }
    else
    {
        // Try to initialize
        if((int)mCurrentFrame.mvKeys.size()<=100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            return;
        }

        // Find correspondences
        ORBmatcher matcher(0.9,true);
        int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);

        // Check if there are enough correspondences
        if(nmatches<100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            return;
        }

        cv::Mat Rcw; // Current Camera Rotation
        cv::Mat tcw; // Current Camera Translation
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        if(mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
        {
            for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
            {
                if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                {
                    mvIniMatches[i]=-1;
                    nmatches--;
                }
            }

            // Set Frame Poses
            mInitialFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
            cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
            Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
            tcw.copyTo(Tcw.rowRange(0,3).col(3));
            mCurrentFrame.SetPose(Tcw);

            CreateInitialMapMonocular();
        }
    }
}

void Tracking::CreateInitialMapMonocular()
{
    // Create KeyFrames
    KeyFrame* pKFini = new KeyFrame(mInitialFrame,mpMap,mpKeyFrameDB);
    KeyFrame* pKFcur = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    mpMap->AddKeyFrame(pKFini);
    mpMap->AddKeyFrame(pKFcur);

    // Create MapPoints and associate to keyframes
    for(size_t i=0; i<mvIniMatches.size();i++)
    {
        if(mvIniMatches[i]<0)
            continue;

        //Create MapPoint.
        cv::Mat worldPos(mvIniP3D[i]);

        MapPoint* pMP = new MapPoint(worldPos,pKFcur,mpMap);

        pKFini->AddMapPoint(pMP,i);
        pKFcur->AddMapPoint(pMP,mvIniMatches[i]);

        pMP->AddObservation(pKFini,i);
        pMP->AddObservation(pKFcur,mvIniMatches[i]);

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();

        //Fill Current Frame structure
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

        //Add to Map
        mpMap->AddMapPoint(pMP);
    }

    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    // Bundle Adjustment
    cout << "New Map created with " << mpMap->MapPointsInMap() << " points" << endl;

    Optimizer::GlobalBundleAdjustemnt(mpMap,20);

    // Set median depth to 1
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth = 1.0f/medianDepth;

    if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<100)
    {
        cout << "Wrong initialization, reseting..." << endl;
        Reset();
        return;
    }

    // Scale initial baseline
    cv::Mat Tc2w = pKFcur->GetPose();
    Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
    for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            MapPoint* pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
        }
    }

    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);

    mCurrentFrame.SetPose(pKFcur->GetPose());
    mnLastKeyFrameId=mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints=mpMap->GetAllMapPoints();
    mpReferenceKF = pKFcur;
    mCurrentFrame.mpReferenceKF = pKFcur;

    mLastFrame = Frame(mCurrentFrame);

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

	if (mpViewer != NULL) mpViewer->SetCurrentCameraPose(mCurrentFrame.mTcw);

    mpMap->mvpKeyFrameOrigins.push_back(pKFini);

    mState=OK;
}

void Tracking::CheckReplacedInLastFrame()
{
    for(int i =0; i<mLastFrame.N; i++)
    {
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];

        if(pMP)
        {
            MapPoint* pRep = pMP->GetReplaced();
            if(pRep)
            {
                mLastFrame.mvpMapPoints[i] = pRep;
            }
        }
    }
}


bool Tracking::TrackReferenceKeyFrame()
{
    // Compute Bag of Words vector
    mCurrentFrame.ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7,true);
    vector<MapPoint*> vpMapPointMatches;

    int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);

    if(nmatches<15)
        return false;

    mCurrentFrame.mvpMapPoints = vpMapPointMatches;
    mCurrentFrame.SetPose(mLastFrame.mTcw);

    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }

    return nmatchesMap>=10;
}

void Tracking::UpdateLastFrame()
{
    // Update pose according to reference keyframe
    KeyFrame* pRef = mLastFrame.mpReferenceKF;
    cv::Mat Tlr = mlRelativeFramePoses.back();

    mLastFrame.SetPose(Tlr*pRef->GetPose());

    if(mnLastKeyFrameId==mLastFrame.mnId || mSensor==System::MONOCULAR)
        return;

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    vector<pair<float,int> > vDepthIdx;
    vDepthIdx.reserve(mLastFrame.N);
    for(int i=0; i<mLastFrame.N;i++)
    {
        float z = mLastFrame.mvDepth[i];
        if(z>0)
        {
            vDepthIdx.push_back(make_pair(z,i));
        }
    }

    if(vDepthIdx.empty())
        return;

    sort(vDepthIdx.begin(),vDepthIdx.end());

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    int nPoints = 0;
    for(size_t j=0; j<vDepthIdx.size();j++)
    {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        MapPoint* pMP = mLastFrame.mvpMapPoints[i];
        if(!pMP)
            bCreateNew = true;
        else if(pMP->Observations()<1)
        {
            bCreateNew = true;
        }

        if(bCreateNew)
        {
            cv::Mat x3D = mLastFrame.UnprojectStereo(i);
            MapPoint* pNewMP = new MapPoint(x3D,mpMap,&mLastFrame,i);

            mLastFrame.mvpMapPoints[i]=pNewMP;

            mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        }
        else
        {
            nPoints++;
        }

        if(vDepthIdx[j].first>mThDepth && nPoints>100)
            break;
    }
}

bool Tracking::TrackWithMotionModel()
{
   ORBmatcher matcher(0.9,true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points
    UpdateLastFrame();

    mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);

    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

    // Project points seen in previous frame
    int th;
    if(mSensor!=System::STEREO)
        th=15;
    else
        th=7;
    int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR);
	//cerr << "In TrackWithMotionModel, first match " << nmatches << endl;
	//cerr << " " << nmatches;// << " " << mVelocity << " " << mLastFrame.mTcw;

    // If few matches, uses a wider window search
    if(nmatches<20)
    {
        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
        nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,2*th,mSensor==System::MONOCULAR);
    }

    if(nmatches<20)
        return false;

    // Optimize frame pose with all matches
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }    

    if(mbOnlyTracking)
    {
        mbVO = nmatchesMap<10;
        return nmatches>20;
    }

    return nmatchesMap>=10;
}

bool Tracking::TrackLocalMap()
{
    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.

    UpdateLocalMap();

    SearchLocalPoints();

    // Optimize Pose
    Optimizer::PoseOptimization(&mCurrentFrame);
    mnMatchesInliers = 0;

    // Update MapPoints Statistics
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(!mCurrentFrame.mvbOutlier[i])
            {
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                if(!mbOnlyTracking)
                {
                    if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                        mnMatchesInliers++;
                }
                else
                    mnMatchesInliers++;
            }
            else if(mSensor==System::STEREO)
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);

        }
    }

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
        return false;

    if(mnMatchesInliers<30)
        return false;
    else
        return true;
}


bool Tracking::NeedNewKeyFrame()
{
    if(mbOnlyTracking)
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;

    const int nKFs = mpMap->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames)
        return false;

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if(nKFs<=2)
        nMinObs=2;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Stereo & RGB-D: Ratio of close "matches to map"/"total matches"
    // "total matches = matches to map + visual odometry matches"
    // Visual odometry matches will become MapPoints if we insert a keyframe.
    // This ratio measures how many MapPoints we could create if we insert a keyframe.
    int nMap = 0;
    int nTotal= 0;
    if(mSensor!=System::MONOCULAR)
    {
        for(int i =0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvDepth[i]>0 && mCurrentFrame.mvDepth[i]<mThDepth)
            {
                nTotal++;
                if(mCurrentFrame.mvpMapPoints[i])
                    if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                        nMap++;
            }
        }
    }
    else
    {
        // There are no visual odometry matches in the monocular case
        nMap=1;
        nTotal=1;
    }

    const float ratioMap = (float)nMap/fmax(1.0f,nTotal);

    // Thresholds
    float thRefRatio = 0.75f;
    if(nKFs<2)
        thRefRatio = 0.4f;

    if(mSensor==System::MONOCULAR)
        thRefRatio = 0.9f;

    float thMapRatio = 0.35f;
    if(mnMatchesInliers>300)
        thMapRatio = 0.20f;

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = (mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames && bLocalMappingIdle);
    //Condition 1c: tracking is weak
    const bool c1c =  mSensor!=System::MONOCULAR && (mnMatchesInliers<nRefMatches*0.25 || ratioMap<0.3f) ;
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 = ((mnMatchesInliers<nRefMatches*thRefRatio|| ratioMap<thMapRatio) && mnMatchesInliers>15);

    if((c1a||c1b||c1c)&&c2)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if(bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            if(mSensor!=System::MONOCULAR)
            {
                if(mpLocalMapper->KeyframesInQueue()<3)
                    return true;
                else
                    return false;
            }
            else
                return false;
        }
    }
    else
        return false;
}

void Tracking::CreateNewKeyFrame()
{
    if(!mpLocalMapper->SetNotStop(true))
        return;

    KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;

    if(mSensor!=System::MONOCULAR)
    {
        mCurrentFrame.UpdatePoseMatrices();

        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        vector<pair<float,int> > vDepthIdx;
        vDepthIdx.reserve(mCurrentFrame.N);
        for(int i=0; i<mCurrentFrame.N; i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                vDepthIdx.push_back(make_pair(z,i));
            }
        }

        if(!vDepthIdx.empty())
        {
            sort(vDepthIdx.begin(),vDepthIdx.end());

            int nPoints = 0;
            for(size_t j=0; j<vDepthIdx.size();j++)
            {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;

                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(!pMP)
                    bCreateNew = true;
                else if(pMP->Observations()<1)
                {
                    bCreateNew = true;
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                }

                if(bCreateNew)
                {
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                    MapPoint* pNewMP = new MapPoint(x3D,pKF,mpMap);
                    pNewMP->AddObservation(pKF,i);
                    pKF->AddMapPoint(pNewMP,i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpMap->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                    nPoints++;
                }
                else
                {
                    nPoints++;
                }

                if(vDepthIdx[j].first>mThDepth && nPoints>100)
                    break;
            }
        }
    }

    mpLocalMapper->InsertKeyFrame(pKF);

    mpLocalMapper->SetNotStop(false);

    //mpPointCloudMapping->PointCloudMapping::insertKeyFrame( pKF, this->mImRGB, this->mImDepth );
    mpPointCloudMapping->insertKeyFrame( pKF, this->mImRGB, this->mImDepth );

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
}

void Tracking::CreateNewKeyFrame(string tat)
{
    if(!mpLocalMapper->SetNotStop(true))
        return;
    string dsave;
    string dasave;
    stringstream fsave;
    stringstream fdsave;
    stringstream sty;
    KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;

    if(mSensor!=System::MONOCULAR)
    {
        mCurrentFrame.UpdatePoseMatrices();

        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        vector<pair<float,int> > vDepthIdx;
        vDepthIdx.reserve(mCurrentFrame.N);
        for(int i=0; i<mCurrentFrame.N; i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                vDepthIdx.push_back(make_pair(z,i));
            }
        }

        if(!vDepthIdx.empty())
        {
            sort(vDepthIdx.begin(),vDepthIdx.end());

            int nPoints = 0;
            for(size_t j=0; j<vDepthIdx.size();j++)
            {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;

                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(!pMP)
                    bCreateNew = true;
                else if(pMP->Observations()<1)
                {
                    bCreateNew = true;
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                }

                if(bCreateNew)
                {
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                    MapPoint* pNewMP = new MapPoint(x3D,pKF,mpMap);
                    pNewMP->AddObservation(pKF,i);
                    pKF->AddMapPoint(pNewMP,i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpMap->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                    nPoints++;
                }
                else
                {
                    nPoints++;
                }

                if(vDepthIdx[j].first>mThDepth && nPoints>100)
                    break;
            }
        }
    }

    mpLocalMapper->InsertKeyFrame(pKF);

    mpLocalMapper->SetNotStop(false);
    sty<<"/home/jinglun/"<<tat<<"/";
    string acd;
    sty>>acd;
    cout<<acd<<endl;
    fsave<<acd<<"KeyFrame_ID"<<pKF->mnId <<".jpeg";
    fdsave<<acd<<"KEYFRAME_DEPTH_ED"<<pKF->mnId<<".jpeg";
    fsave>>dsave;
    fdsave>>dasave;

    cout<<dsave<<endl;
    cout<<dasave<<endl;

    cv::imwrite(dsave,this->mImRGB);
    cv::imwrite(dasave,this->mImDepth);
    //mpPointCloudMapping->PointCloudMapping::insertKeyFrame( pKF, this->mImRGB, this->mImDepth );
    mpPointCloudMapping->insertKeyFrame( pKF, this->mImRGB, this->mImDepth );

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
}

void Tracking::SearchLocalPoints()
{
    // Do not search map points already matched
    for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = static_cast<MapPoint*>(NULL);
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                pMP->mbTrackInView = false;
            }
        }
    }

    int nToMatch=0;

    // Project points in frame and check its visibility
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if(pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        if(mCurrentFrame.isInFrustum(pMP,0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++;
        }
    }

    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);
        int th = 1;
        if(mSensor==System::RGBD)
            th=3;
        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
            th=5;
        matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th);
    }
}

void Tracking::UpdateLocalMap()
{
    // This is for visualization
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    // Update
    UpdateLocalKeyFrames();
    UpdateLocalPoints();
}

void Tracking::UpdateLocalPoints()
{
    mvpLocalMapPoints.clear();

    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        KeyFrame* pKF = *itKF;
        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                continue;
            if(!pMP->isBad())
            {
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
            }
        }
    }
}


void Tracking::UpdateLocalKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
    map<KeyFrame*,int> keyframeCounter;
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if(!pMP->isBad())
            {
                const map<KeyFrame*,size_t> observations = pMP->GetObservations();
                for(map<KeyFrame*,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                    keyframeCounter[it->first]++;
            }
            else
            {
                mCurrentFrame.mvpMapPoints[i]=NULL;
            }
        }
    }

    if(keyframeCounter.empty())
        return;

    int max=0;
    KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFrame* pKF = it->first;

        if(pKF->isBad())
            continue;

        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }

        mvpLocalKeyFrames.push_back(it->first);
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }


    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if(mvpLocalKeyFrames.size()>80)
            break;

        KeyFrame* pKF = *itKF;

        const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        const set<KeyFrame*> spChilds = pKF->GetChilds();
        for(set<KeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            KeyFrame* pChildKF = *sit;
            if(!pChildKF->isBad())
            {
                if(pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        KeyFrame* pParent = pKF->GetParent();
        if(pParent)
        {
            if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                break;
            }
        }

    }

    if(pKFmax)
    {
        mpReferenceKF = pKFmax;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}

bool Tracking::Relocalization()
{
    // Compute Bag of Words Vector
    mCurrentFrame.ComputeBoW();

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);

    if(vpCandidateKFs.empty())
        return false;

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75,true);

    vector<PnPsolver*> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates=0;

    for(int i=0; i<nKFs; i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
            if(nmatches<15)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                PnPsolver* pSolver = new PnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9,true);

    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nKFs; i++)
        {
            if(vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            PnPsolver* pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(!Tcw.empty())
            {
                Tcw.copyTo(mCurrentFrame.mTcw);

                set<MapPoint*> sFound;

                const int np = vbInliers.size();

                for(int j=0; j<np; j++)
                {
                    if(vbInliers[j])
                    {
                        mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame.mvpMapPoints[j]=NULL;
                }

                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                if(nGood<10)
                    continue;

                for(int io =0; io<mCurrentFrame.N; io++)
                    if(mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL);

                // If few inliers, search by projection in a coarse window and optimize again
                if(nGood<50)
                {
                    int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

                    if(nadditional+nGood>=50)
                    {
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if(nGood>30 && nGood<50)
                        {
                            sFound.clear();
                            for(int ip =0; ip<mCurrentFrame.N; ip++)
                                if(mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

                            // Final optimization
                            if(nGood+nadditional>=50)
                            {
                                nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                for(int io =0; io<mCurrentFrame.N; io++)
                                    if(mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.mvpMapPoints[io]=NULL;
                            }
                        }
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                if(nGood>=50)
                {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        return false;
    }
    else
    {
        mnLastRelocFrameId = mCurrentFrame.mnId;
        return true;
    }

}

void Tracking::Reset()
{
    mpViewer->RequestStop();

    cout << "System Reseting" << endl;
    while(!mpViewer->isStopped())
        usleep(3000);

    // Reset Local Mapping
    cout << "Reseting Local Mapper...";
    mpLocalMapper->RequestReset();
    cout << " done" << endl;

    // Reset Loop Closing
    cout << "Reseting Loop Closing...";
    mpLoopClosing->RequestReset();
    cout << " done" << endl;

    // Clear BoW Database
    cout << "Reseting Database...";
    mpKeyFrameDB->clear();
    cout << " done" << endl;

    // Clear Map (this erase MapPoints and KeyFrames)
    mpMap->clear();

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;

    if(mpInitializer)
    {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
    }

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();

    mpViewer->Release();
}

void Tracking::ChangeCalibration(const string &strSettingPath)
{
  Camera::Load(strSettingPath);
  Frame::mbInitialComputations = true;
}

void Tracking::InformOnlyTracking(const bool &flag)
{
    mbOnlyTracking = flag;
}


} //namespace ORB_SLAM
