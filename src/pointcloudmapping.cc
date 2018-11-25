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
        lastKeyframeSize = N;
    }
}

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

