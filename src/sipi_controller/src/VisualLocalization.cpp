#include "VisualLocalization.h"
#include <iostream>
#include <angles/angles.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <ros/ros.h>
using namespace std;

/*** constructor.  needs handle to ros interface for messages */
CvisualLocalization::CvisualLocalization(string _name, ros::NodeHandle nh) 
{
	rosNode = nh;
	name = _name;
	apriltagObservedSubscriber = rosNode.subscribe(name+"/targets", 1, 
			&CvisualLocalization::apriltagObservedHandler, this);

	visualLocationPublish = rosNode.advertise<nav_msgs::Odometry>(
			name + "/visualLocation", 1);
	/*
		 publishTimer = rosNode.createTimer(
		 ros::Duration(0.1), &CvisualLocalization::publishTimerHandler, this);
	 */
	parent_frame = name + "/arena";
	child_frame = name + "/map";

	visualLocationMsg.header.frame_id = parent_frame;
	visualLocationMsg.child_frame_id = child_frame;
	visualLocationMsg.pose.covariance[0] = 0.25;
	visualLocationMsg.pose.covariance[7] = 0.25;
	visualLocationMsg.pose.covariance[35] = 0.1;

	tf::Quaternion q;
	tParentToChild.setOrigin( tf::Vector3(
				0.0, 0.0, 0.0) );
	q.setRPY(0, 0, 0);
	tParentToChild.setRotation(q);
	//	localized = homeSeen = homeVisible = updateLocalization = false;
	//	localizeCount = 0;	

	loopTimer = rosNode.createTimer(
			ros::Duration(0.01),
			&CvisualLocalization::loopTimerHandler, this);

	publishTransform = true;
	// home tag's pose is at 0,0,0
	homeTagPose.position.x = homeTagPose.position.y = 
		homeTagPose.position.z = 0.0; 
	// at least in sim, they are rotated so that X is pointing down 
	homeTagPose.orientation = tf::createQuaternionMsgFromYaw(-M_PI_2);
}

void CvisualLocalization::loopTimerHandler(const ros::TimerEvent& event)
{
	visualLocationMsg.header.stamp = ros::Time::now();
	visualLocationPublish.publish(visualLocationMsg);
	/*
		 ROS_INFO_STREAM("Published transform from " << visualLocationMsg.header.frame_id <<
		 " to " << visualLocationMsg.child_frame_id );
	 */
	if(publishTransform) {
		tf_pub.sendTransform(tf::StampedTransform(tParentToChild, ros::Time::now(), 
					parent_frame, child_frame));
	}
}

void CvisualLocalization::apriltagObservedHandler(
		const apriltags_ros::AprilTagDetectionArray::ConstPtr& message) 
{
	size_t n = message->detections.size();
	if(n == 0) return;
	int homeCount = 0; // how many home tags visible
	geometry_msgs::Pose avgPose;
	for(size_t i=0;i<n;i++) {
		if(message->detections[i].id == 256) {
			homeCount++;
		}
	}
	if(homeCount > 0) {
		// get transform from landmark to child frame
		tf::StampedTransform tTagToChild;
		try {
			listener.lookupTransform(name + "/tag_256", child_frame,  
					ros::Time(0), tTagToChild); 
		} catch (tf::TransformException &ex) {
			ROS_ERROR("%s",ex.what());
			return;
		}
		tf::Stamped<tf::Pose> tParentToTag;
		tf::poseMsgToTF(homeTagPose, tParentToTag);
		// calculate tf from world to odom through tag
		tParentToChild = tParentToTag*tTagToChild;
		// use this in localization msg
		// force to 2D plane
		tf::Vector3 origin = tParentToChild.getOrigin();
		origin.setZ(0.0);
		tParentToChild.setOrigin(origin);
		double r,p,y;
		tf::Matrix3x3(tParentToChild.getRotation()).getRPY(r,p,y);
		tf::Quaternion q;
		q.setRPY(0,0,y);
		tParentToChild.setRotation(q);
		tf::poseTFToMsg(tParentToChild, visualLocationMsg.pose.pose);
		/*
		ROS_INFO_STREAM("LM tParentToChild= "<< 
				tTagToChild.getOrigin().x() << ", " <<
				tTagToChild.getOrigin().y() << ", " <<
				tTagToChild.getOrigin().z() << " : " <<
				tParentToTag.getOrigin().x() << ", " <<
				tParentToTag.getOrigin().y() << ", " <<
				tParentToTag.getOrigin().z() << " : " <<
				tParentToChild.getOrigin().x() << ", " <<
				tParentToChild.getOrigin().y() << ", " <<
				tParentToChild.getOrigin().z());
		*/
	}
}
