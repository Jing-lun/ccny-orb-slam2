/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */

#include "pointcloudmapping.h"
#include <KeyFrame.h>
#include "MapPoint.h"
#include "Camera.h"
#include <opencv2/highgui/highgui.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/projection_matrix.h>
#include <tf/transform_datatypes.h>
#include <pcl/io/pcd_io.h>
#include "Converter.h"
#include "Map.h"
#include <iostream>
#include <string>
#include <set>
#include <fstream>
#include <iterator>
#include <vector>
#include <mutex>

<<<<<<< HEAD
=======

>>>>>>> d44933e8356f6014b6c205e747bf5c9be8dd54d6
PointCloudMapping::PointCloudMapping(double resolution_)
{
    cout<<"--This is PointCloud Constructor"<<endl;
    this->resolution = resolution_;
    voxel.setLeafSize( resolution, resolution, resolution );
    //voxel.setLeafSize( 0.02f, 0.02f, 0.02f );
    //PointCloud::Ptr globalMap( new PointCloud() );
    globalMap = boost::make_shared< PointCloud >( );
    //mpviewer = new PointCloudMapping::viewer();
    //mptViewer = new thread(&PointCloudMapping::viewer::Run, mpViewer);
    viewerThread = make_shared<thread>( bind(&PointCloudMapping::viewer, this ) );
<<<<<<< HEAD
    // globalpointcloud.reserve(1000000);
    newpointcloud.reserve(1000000);
=======
>>>>>>> d44933e8356f6014b6c205e747bf5c9be8dd54d6
}

void PointCloudMapping::shutdown()
{
    cout<<"--PointCloud Viewer ShutDown!"<<endl;
    {
        unique_lock<mutex> lck(shutDownMutex);
        shutDownFlag = true;
        keyFrameUpdated.notify_one();
    }
    viewerThread->join();
}

void PointCloudMapping::insertKeyFrame(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
    cout<<"receive a keyframe, id = "<<kf->mnId<<endl;
    unique_lock<mutex> lck(keyframeMutex);
    keyframes.push_back( kf );
    colorImgs.push_back( color.clone() );
    depthImgs.push_back( depth.clone() );
    
    keyFrameUpdated.notify_one();
}

pcl::PointCloud< PointCloudMapping::PointT >::Ptr PointCloudMapping::generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
    PointCloud::Ptr tmp( new PointCloud() );
<<<<<<< HEAD
    Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( kf->GetPose() );
=======
>>>>>>> d44933e8356f6014b6c205e747bf5c9be8dd54d6
    // point cloud is null ptr
    for ( int m=0; m<depth.rows; m+=3 )
    {
        for ( int n=0; n<depth.cols; n+=3 )
        {
            float d = depth.ptr<float>(m)[n];
            if (d < 0.01 || d>10)
                continue;
            PointT p;
            p.z = d;
            p.x = ( n - Camera::cx) * p.z / Camera::fx;
            p.y = ( m - Camera::cy) * p.z / Camera::fy;
            
            p.b = color.ptr<uchar>(m)[n*3];
            p.g = color.ptr<uchar>(m)[n*3+1];
            p.r = color.ptr<uchar>(m)[n*3+2];
                
            tmp->points.push_back(p);
<<<<<<< HEAD

            // Eigen::Vector3d newpoint;
            // newpoint[2] = double(d);
            // newpoint[0] = ( n - Camera::cx ) * newpoint[2] / Camera::fx;
            // newpoint[1] = ( m - Camera::cy ) * newpoint[2] / Camera::fy;
            // Eigen::Vector3d pointWorld = T * newpoint;

            // Vector6d newp;
            // newp.head<3>() = pointWorld;
            // newp[5] = color.data[m * color.step + n * color.channels()];   // blue
            // newp[4] = color.data[m * color.step + n * color.channels() + 1]; // green
            // newp[3] = color.data[m * color.step + n * color.channels() + 2]; // red
            // newpointcloud.push_back(newp);
        }
    }
    
    PointCloud::Ptr cloud(new PointCloud);
    pcl::transformPointCloud( *tmp, *cloud, T.inverse().matrix());
    cloud->is_dense = false;

    // showPointCloud(newpointcloud);
    
    cout<<"generate point cloud for kf "<<kf->mnId<<", size="<<tmp->points.size()<<endl;
    return cloud;
}

// vector<Vector6d, Eigen::aligned_allocator<Vector6d>> PointCloudMapping::generatePointcloudMap(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
void PointCloudMapping::generatePointcloudMap(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
    // vector<Vector6d, Eigen::aligned_allocator<Vector6d>> newpointcloud;
    Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( kf->GetPose() );
    // point cloud is null ptr
    for ( int m=0; m<depth.rows; m+=3 )
    {
        for ( int n=0; n<depth.cols; n+=3 )
        {
            float d = depth.ptr<float>(m)[n];
            if (d < 0.01 || d>10)
                continue;
            
            Eigen::Vector3d newpoint;
            newpoint[2] = double(d);
            newpoint[0] = ( n - Camera::cx ) * newpoint[2] / Camera::fx;
            newpoint[1] = ( m - Camera::cy ) * newpoint[2] / Camera::fy;
            Eigen::Vector3d pointWorld = T * newpoint;

            Vector6d newp;
            newp.head<3>() = pointWorld;
            newp[5] = color.data[m * color.step + n * color.channels()];   // blue
            newp[4] = color.data[m * color.step + n * color.channels() + 1]; // green
            newp[3] = color.data[m * color.step + n * color.channels() + 2]; // red
            newpointcloud.push_back(newp);
        }
    }
    // showPointCloud(newpointcloud);
    // return newpointcloud;

}

void PointCloudMapping::viewer()
{
    pcl::visualization::CloudViewer viewer("viewer");
    while(1)
    {
=======
        }
    }
    
    Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( kf->GetPose() );
    PointCloud::Ptr cloud(new PointCloud);
    pcl::transformPointCloud( *tmp, *cloud, T.inverse().matrix());
    cloud->is_dense = false;
    
    cout<<"generate point cloud for kf "<<kf->mnId<<", size="<<cloud->points.size()<<endl;
    return cloud;
}


void PointCloudMapping::viewer()
{
    //viewerThread->join();
    //cout<<"--PointCloud Viewer starts!"<<endl;
    pcl::visualization::CloudViewer viewer("viewer");
	PointCloud::Ptr tmpcloud(new PointCloud());
	pass.setFilterFieldName("z");
    pass.setFilterLimits( 0.0, 4.0 ); //4m以上就不要了

    while(1)
    {
        //cout<<"PointCloud Viewer has joined in a loop"<<endl;
>>>>>>> d44933e8356f6014b6c205e747bf5c9be8dd54d6
        {
            unique_lock<mutex> lck_shutdown( shutDownMutex );
            if (shutDownFlag)
            {
                break;
            }
        }
        {
            //cout<<"PointCloud Viewer has joined in generatePointCloud"<<endl;
            unique_lock<mutex> lck_keyframeUpdated( keyFrameUpdateMutex );
            keyFrameUpdated.wait( lck_keyframeUpdated );
        }
        
        // keyframe is updated 
        size_t N=0;
        {
            unique_lock<mutex> lck( keyframeMutex );
            N = keyframes.size();
        }
        
<<<<<<< HEAD
        // for ( size_t i=lastKeyframeSize; i<N ; i++ )
        // {
        //     cout<<"--PointCloud Viewer has joined in generatePointCloud"<<endl;
        //     generatePointcloudMap(keyframes[i], colorImgs[i], depthImgs[i]);
        //     cout << "newpointcloud.size() = " << newpointcloud.size() << endl;
        // }

        // showPointCloud(newpointcloud);
        // lastKeyframeSize = N;

        for ( size_t i=lastKeyframeSize; i<N ; i++ )
        {
            cout<<"--PointCloud Viewer has joined in generatePointCloud"<<endl;
            PointCloud::Ptr p = generatePointCloud( keyframes[i], colorImgs[i], depthImgs[i] );
            *globalMap += *p;
            p->clear();
        }
        globalMap->height=1;
        globalMap->width=globalMap->points.size();
        globalMap->is_dense=false;
        pcl::io::savePCDFile( "global_cloud.pcd", *globalMap );
        boost::this_thread::sleep(boost::posix_time::microseconds (100000));
        viewer.showCloud(globalMap);
        cout<<"show globalMap size="<<globalMap->points.size()<<endl;
        // showPointCloud(newpointcloud);
=======
        for ( size_t i=lastKeyframeSize; i<N ; i++ )
        {
            //cout<<"--PointCloud Viewer has joined in generatePointCloud"<<endl;
            PointCloud::Ptr p = generatePointCloud( keyframes[i], colorImgs[i], depthImgs[i] );
            //以下为20170929新增
            PointCloud::Ptr tmp(new PointCloud());
            voxel.setInputCloud( p );
            voxel.filter( *tmpcloud );
            pass.setInputCloud( tmpcloud );
            pass.filter( *p );
            //pcl::transformPointCloud( *tmp, *p, T.inverse().matrix());
            *globalMap += *p;
            p->clear();
            tmp->clear();
            //以上为新增内容
        }
        //voxel.setInputCloud( globalMap );
        //voxel.filter( *p );
        //pcl::io::savePCDFile( "./orbcloud.pcd", *p);
        //cout<<"show tmp size="<<tmp->points.size()<<endl;

        PointCloud::Ptr filtercloud(new PointCloud());
        voxel.setInputCloud( globalMap );
        voxel.filter( *filtercloud );
		globalMap->swap( *filtercloud );

        //globalMap->height=1;
        //globalMap->width=globalMap->points.size();
        //globalMap->is_dense=false;
        //pcl::io::savePCDFile( "/home/jinglun/viorb_config/global_cloud.pcd", *globalMap );
        boost::this_thread::sleep(boost::posix_time::microseconds (100000));
        viewer.showCloud(globalMap);
        cout<<"show globalMap size="<<globalMap->points.size()<<endl;
>>>>>>> d44933e8356f6014b6c205e747bf5c9be8dd54d6
        lastKeyframeSize = N;
    }
}

<<<<<<< HEAD
void PointCloudMapping::showPointCloud(const vector<Vector6d> &pointcloud) {
    if (pointcloud.empty()) {
        cerr << "Point cloud is empty!" << endl;
        return;
    }

    pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
        .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glPointSize(2);
        glBegin(GL_POINTS);
        for (auto &p: pointcloud) {
            glColor3d(p[3] / 255.0, p[4] / 255.0, p[5] / 255.0);
            glVertex3d(p[0], p[1], p[2]);
        }
        glEnd();
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
    return;
}
=======
void PointCloudMapping::savePointCloudToPcd()
{
	    cout<<"Saving the cloud."<<endl;
	    globalMap->height = 1;
	    globalMap->width = globalMap->points.size();
	    globalMap->is_dense = false;
	    pcl::io::savePCDFile( "/home/jinglun/viorb_config/global_cloud.pcd", *globalMap);
	    globalMap->points.clear();
	    cout<<"Point cloud saved."<<endl;
}

>>>>>>> d44933e8356f6014b6c205e747bf5c9be8dd54d6
