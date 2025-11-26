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
#include "KeyFrame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include <opencv2/highgui/highgui.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <chrono>


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

pcl::PointCloud< PointT >::Ptr PointCloudMapping::generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
    PointCloud::Ptr tmp( new PointCloud() );

    for ( int m=0; m<depth.rows; m+=3 )
    {
        for ( int n=0; n<depth.cols; n+=3 )
        {
            float d = depth.ptr<float>(m)[n];
            if (d < 0.01)
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

struct UserData {
    bool followCam = true;
    double dist = 7.;
    double movx = 0.;
    double movy = 0.;
};

void keyboardEvent(const pcl::visualization::KeyboardEvent &event, void *user_data)
{
    if (!event.keyDown()) return;

	std::cout << "key press!" << std::endl;

    auto data = static_cast<UserData*>(user_data);
    double mov_amount = 0.3;

    if (event.getKeySym() == "v") {

        data->followCam = data->followCam ? false : true;
        data->movx = 0.;
        data->movy = 0.;
		data->dist = 7.;
        std::cout << "Camera moved to passed pose!" << std::endl;
    }
    else if (event.getKeySym() == "Down") {

        data->dist += 1;
        std::cout << "Camera moved to further!" << std::endl;
    }
    else if (event.getKeySym() == "Up") {

        data->dist -= 1;
        std::cout << "Camera moved to nearer!" << std::endl;
    }
    else if (event.getKeySym() == "w") {

        data->movx += mov_amount;
        std::cout << "Camera moved to up!" << std::endl;
    }
    else if (event.getKeySym() == "s") {

        data->movx -= mov_amount;
        std::cout << "Camera moved to down!" << std::endl;
    }
    else if (event.getKeySym() == "a") {

        data->movy += mov_amount;
        std::cout << "Camera moved to left!" << std::endl;
    }
    else if (event.getKeySym() == "d") {

        data->movy -= mov_amount;
        std::cout << "Camera moved to right!" << std::endl;
    }
}


void PointCloudMapping::viewer()
{
    pcl::visualization::PCLVisualizer viewer("viewer");
    UserData userdata;

    viewer.registerKeyboardCallback(keyboardEvent, (void*)&userdata);

 //   pcl::StatisticalOutlierRemoval<PointT> statistical_filter;
	//statistical_filter.setMeanK(40);
	//statistical_filter.setStddevMulThresh(1.0);

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
            Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( keyframes[i]->GetPose() );
            PointCloud::Ptr tmp = generatePointCloud( keyframes[i], colorImgs[i], depthImgs[i]);

            *(globalMap) += *tmp;

            // camera → world transform
			Eigen::Isometry3d twc = T.inverse();

			//// camera center
			Eigen::Vector3d cam_pos = twc.translation();

			//// camera forward (look direction)
			Eigen::Vector3d forward = twc.rotation() * Eigen::Vector3d(0, 0, 1);

			Eigen::Vector3d cam_pos_far = cam_pos - forward.normalized() * userdata.dist;

			Eigen::Vector3d look_at = cam_pos + forward;

			//// camera up
			Eigen::Vector3d up_dir = twc.rotation() * Eigen::Vector3d(0, -1, 0);

            if (i == 0)
                viewer.addPointCloud(tmp, "cloud");
            else
                viewer.updatePointCloud(tmp, "cloud");

            if (userdata.followCam) {
				viewer.setCameraPosition(
					cam_pos_far.x() + userdata.movy, cam_pos_far.y() + userdata.movx, cam_pos_far.z(),
					look_at.x(), look_at.y(), look_at.z(),
					up_dir.x(), up_dir.y(), up_dir.z()
				);
            }

			viewer.spinOnce(40);
        }
		lastKeyframeSize = N;
    }

    cout << "saving map ..." << endl;
    pcl::io::savePCDFileBinary("map.pcd", *globalMap);
    cout << "saved ..." << endl;
}
