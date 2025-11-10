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
#include <opencv2/highgui/highgui.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <chrono>
#include "Converter.h"

PointCloudMapping::PointCloudMapping(double resolution_)
{
    this->resolution = resolution_;
    voxel.setLeafSize( resolution, resolution, resolution);
    globalMap = std::make_shared< PointCloud >( );
    
    viewerThread = make_shared<thread>( bind(&PointCloudMapping::viewer, this ) );
}

void PointCloudMapping::shutdown()
{
    {
        unique_lock<mutex> lock(shutDownMutex);
        shutDownFlag = true;
        keyFrameUpdated.notify_one();
    }
    viewerThread->join();
}

void PointCloudMapping::insertKeyFrame(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
    unique_lock<mutex> lock(keyframeMutex);
    keyframes.push_back( kf );
    colorImgs.push_back( color.clone() );
    depthImgs.push_back( depth.clone() );
    
    keyFrameUpdated.notify_one();
}

void PointCloudMapping::AddGBAKeyFrameId(int nmId) 
{
    unique_lock<mutex> lock(GBAKFmutex);
    mvGBAKFs.insert(nmId);
}

void PointCloudMapping::setGBAState(GBAState s){
	unique_lock<std::mutex> lock(GBAMutex);
    meGBAState = s;
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
            p.x = ( n - kf->cx) * p.z / kf->fx;
            p.y = ( m - kf->cy) * p.z / kf->fy;
            
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

    return cloud;
}


void PointCloudMapping::viewer()
{
    pcl::visualization::CloudViewer viewer("viewer");

    // voxel filter
	pcl::VoxelGrid<PointT> voxel_filter;
	double resolution = 0.015;
	voxel_filter.setLeafSize(resolution, resolution, resolution);

	pcl::StatisticalOutlierRemoval<PointT> statistical_filter;
	statistical_filter.setMeanK(45);
	statistical_filter.setStddevMulThresh(1.0);

    while(1)
    {
        {
            unique_lock<mutex> lck_shutdown( shutDownMutex );
            if (shutDownFlag)
            {
                break;
            }
        }
        
        {
            unique_lock<mutex> lck_keyframeUpdated( keyFrameUpdateMutex );
            keyFrameUpdated.wait( lck_keyframeUpdated );
        }
        
        // keyframe is updated 
        size_t N=0;
        {
            unique_lock<mutex> lock( keyframeMutex );
            N = keyframes.size();
        }
        
        for ( size_t i=lastKeyframeSize; i<N ; i++ )
        {
            PointCloud::Ptr p = generatePointCloud( keyframes[i], colorImgs[i], depthImgs[i] );

			PointCloud::Ptr tmp(new PointCloud());
			statistical_filter.setInputCloud(p);
			statistical_filter.filter(*tmp);

			(*globalMap) += *tmp;
            
            tmp = std::make_shared<PointCloud>();
			voxel_filter.setInputCloud(globalMap);
			voxel_filter.filter(*tmp);
			tmp->swap(*globalMap);
        }

        viewer.showCloud( globalMap );
        lastKeyframeSize = N;
    }


    { 
        unique_lock<std::mutex> lock(GBAMutex);
        if (meGBAState == NOT_ACTIVE) {
			cout << "saving map ..." << endl;
			pcl::io::savePCDFileBinary("map.pcd", *globalMap);
			cout << "saved ..." << endl;
            return;
        }
        else {
			cout << "wait for GBA to finish.." << endl;
			while(meGBAState == RUNNING)
				std::this_thread::sleep_for(std::chrono::seconds(1)); 
        }
    }   

    
    cout << "redraw global map.." << endl;
    globalMap->clear();

	statistical_filter.setMeanK(30);
	resolution = 0.017;
	voxel_filter.setLeafSize(resolution, resolution, resolution);

	for ( size_t i=0; i<keyframes.size(); i++ )
	{
        KeyFrame* pKF = keyframes[i];

        if (mvGBAKFs.find(pKF->mnId) != mvGBAKFs.end()) {
			PointCloud::Ptr p = generatePointCloud( pKF, colorImgs[i], depthImgs[i] );

			PointCloud::Ptr tmp(new PointCloud());
			pcl::StatisticalOutlierRemoval<PointT> statistical_filter;
			statistical_filter.setInputCloud(p);
			statistical_filter.filter(*tmp);

			(*globalMap) += *tmp;
			
			tmp = std::make_shared<PointCloud>();
			voxel_filter.setInputCloud(globalMap);
			voxel_filter.filter(*tmp);
			tmp->swap(*globalMap);

			viewer.showCloud( globalMap );
        }
	}

	PointCloud::Ptr tmp(new PointCloud());
	statistical_filter.setInputCloud(globalMap);
	statistical_filter.filter(*tmp);
	tmp->swap(*globalMap);

    cout << "saving map ..." << endl;
    pcl::io::savePCDFileBinary("map.pcd", *globalMap);
    cout << "saved ..." << endl;
}
