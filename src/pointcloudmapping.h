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

#ifndef POINTCLOUDMAPPING_H
#define POINTCLOUDMAPPING_H

#include "System.h"
#include "Map.h"
#include "MapPoint.h"
#include "Camera.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/common/angles.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/boost.h>
#include <pcl/segmentation/planar_region.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/plane_coefficient_comparator.h>
#include <pcl/segmentation/plane_refinement_comparator.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <condition_variable>
#include <iostream>
#include <string>
#include <set>
#include <fstream>
#include <iterator>
#include <vector>
#include <mutex>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <boost/format.hpp>  // for formating strings
#include <sophus/se3.hpp>
#include <pangolin/pangolin.h>


using namespace ORB_SLAM2;

class PointCloudMapping
{
public:
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    // Vector6d newp;
    
    PointCloudMapping( double resolution_ );
    
    // 插入一个keyframe，会更新一次地图
    void insertKeyFrame( KeyFrame* kf, cv::Mat& color, cv::Mat& depth );
    void shutdown();
    void viewer();
	void savePointCloudToPcd();
    void showPointCloud( const vector<Vector6d> &pointcloud );

    PointCloud::Ptr generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth);
    // vector<Vector6d, Eigen::aligned_allocator<Vector6d>> generatePointcloudMap(KeyFrame* kf, cv::Mat& color, cv::Mat& depth);
    void generatePointcloudMap(KeyFrame* kf, cv::Mat& color, cv::Mat& depth);

    // vector<Vector6d, Eigen::aligned_allocator<Vector6d>> newpointcloud;
    
protected:
    
    
    PointCloud::Ptr globalMap;
    Vector6d tem_p;
    vector<Vector6d> globalpointcloud;
    // vector<Vector6d, Eigen::aligned_allocator<Vector6d>> newpointcloud;
    vector<Vector6d> newpointcloud;
    shared_ptr<thread>  viewerThread;   
    //std::thread* mptviewer
    //shared_ptr<PointCloudMapping> mpviewer;
    bool    shutDownFlag    =false;
    mutex   shutDownMutex;  
    
    condition_variable  keyFrameUpdated;
    mutex               keyFrameUpdateMutex;
    
    // data to generate point clouds
    vector<KeyFrame*>       keyframes;
    vector<cv::Mat>         colorImgs;
    vector<cv::Mat>         depthImgs;
    mutex                   keyframeMutex;
    uint16_t                lastKeyframeSize =0;
    
    double resolution = 0.01;
    pcl::VoxelGrid<PointT>  voxel;
	pcl::PassThrough<PointT> pass;
};

#endif // POINTCLOUDMAPPING_H
