/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam> 
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <string>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Core>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include "lsd_slam_viewer/keyframeGraphMsg.h"
#include "lsd_slam_viewer/keyframeMsg.h"
#include "geometry_msgs/PoseStamped.h"


using namespace std;
using namespace Eigen;


class Visualizer
{
public:
	Visualizer();
	~Visualizer();

	void init();

private:
	void publishPointMarker(
		ros::Publisher pub,
		const Vector3d& pos,
		const string& ns,
		const ros::Time& timestamp,
		int id,
		int action,
		double marker_scale,
		const Vector3d& color);
	void frameCb(lsd_slam_viewer::keyframeMsgConstPtr msg);
	void graphCb(lsd_slam_viewer::keyframeGraphMsgConstPtr msg);
	void poseCb(geometry_msgs::PoseStampedConstPtr msg);
	void debugImgCb(const sensor_msgs::ImageConstPtr& msg);

	ros::Publisher pub_point_;
	ros::Subscriber sub_liveFrames_;
	ros::Subscriber sub_keyFrames_;
	ros::Subscriber sub_graph_;
	ros::Subscriber sub_pose_;
	image_transport::Subscriber sub_it_;
};
