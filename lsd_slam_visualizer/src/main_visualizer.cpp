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
* along with dvo. If not, see <http://www.gnu.org/licenses/>.
*/

#include "boost/thread.hpp"
#include "boost/foreach.hpp"
// #include "sophus/sim3.hpp"
// #include "sophus/se3.hpp"
#include "visualizer.h"


using namespace std;
using namespace Eigen;


Visualizer::Visualizer()
{
}

Visualizer::~Visualizer()
{
}

void Visualizer::init()
{
	ros::NodeHandle local_nh("~");
	ros::NodeHandle nh;

	pub_point_ = local_nh.advertise<visualization_msgs::Marker>("points", 100);

	sub_liveFrames_ = nh.subscribe("lsd_slam/liveframes",1, &Visualizer::frameCb, this);
	// sub_keyFrames_  = nh.subscribe("lsd_slam/keyframes",20, &Visualizer::frameCb, this);
	// sub_graph_      = nh.subscribe("lsd_slam/graph",10, &Visualizer::graphCb, this);
	// sub_pose_       = nh.subscribe("lsd_slam/pose",10, &Visualizer::poseCb, this);
}

// ref. https://github.com/uzh-rpg/rpg_vikit
void Visualizer::publishPointMarker(
	ros::Publisher pub,
	const Vector3d& pos,
	const string& ns,
	const ros::Time& timestamp,
	int id,
	int action,
	double marker_scale,
	const Vector3d& color)
{
	visualization_msgs::Marker msg;
	msg.header.frame_id = "/world";
	msg.header.stamp = timestamp;
	msg.ns = ns;
	msg.id = id;
	msg.type = visualization_msgs::Marker::CUBE;
	msg.action = action; // 0 = add/modify
	msg.scale.x = marker_scale;
	msg.scale.y = marker_scale;
	msg.scale.z = marker_scale;
	msg.color.a = 1.0;
	msg.color.r = color[0];
	msg.color.g = color[1];
	msg.color.b = color[2];
	msg.lifetime = ros::Duration(0.0);
	msg.pose.position.x = pos[0];
	msg.pose.position.y = pos[1];
	msg.pose.position.z = pos[2];
	pub.publish(msg);
}

void Visualizer::frameCb(lsd_slam_viewer::keyframeMsgConstPtr msg)
{
	Vector3d t;
	t[0] = msg->camToWorld[4];
	t[1] = msg->camToWorld[5];
	t[2] = msg->camToWorld[6];

	if(pub_point_.getNumSubscribers() > 0)
	{
		publishPointMarker(
	        pub_point_, 
	        t, 
	        "trajectory",
	        ros::Time::now(), 
	        msg->id, 
	        0, 
	        0.003, 
	        msg->isKeyframe? Vector3d(0.5,0.,0.) : Vector3d(0.,0.,0.5));
	}
}

void Visualizer::graphCb(lsd_slam_viewer::keyframeGraphMsgConstPtr msg)
{
}

void Visualizer::poseCb(geometry_msgs::PoseStampedConstPtr msg)
{
	// Vector3d t;
	// t[0] = msg->camToWorld[4];
	// t[1] = msg->camToWorld[5];
	// t[2] = msg->camToWorld[6];

	// if(pub_point_.getNumSubscribers() > 0)
	// {
	// 	publishPointMarker(
	//         pub_point_, 
	//         t, 
	//         "trajectory",
	//         ros::Time::now(), 
	//         msg->id, 
	//         0, 
	//         0.006, 
	//         msg->isKeyframe? Vector3d(0.5,0.,0.) : Vector3d(0.,0.,0.5));
	// }
}

int main( int argc, char** argv )
{
	ros::init(argc, argv, "lsd_slam_visualizer");
	printf("Started LSD-SLAM Visualizer\n");

	Visualizer Visualizer;
	Visualizer.init();

	ros::spin();
	printf("Done. \n");
}
