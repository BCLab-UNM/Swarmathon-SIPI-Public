#ifndef __VISUAL_LOCALIZATION_H
#define __VISUAL_LOCALIZATION_H
#include <ros/ros.h>
#include <std_msgs/Int16.h>
//#include <swarmie_nodes/targetStatus.h>
//#include "pose.h"

//OpenCV headers
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
//ROS libraries
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include "apriltags_ros/AprilTagDetectionArray.h"

class CvisualLocalization {
	public:
		CvisualLocalization(std::string name, ros::NodeHandle nh);
		/*** call periodically by timer to send updates of the pose messages */
		void loop(void);
		bool isLocalized(void) {return localized;};
		// if true, publish transform directly
		// otherwise another node needs to publish transform based on visualLocationMsg 
		bool publishTransform;
	private:
		geometry_msgs::Pose homeTagPose;
		std::string name;
		// get tag detection arrays
		ros::NodeHandle rosNode;
		ros::Subscriber apriltagObservedSubscriber;
		void apriltagObservedHandler(
				const apriltags_ros::AprilTagDetectionArray::ConstPtr& message);

		// publish odometry message linking parent and child frames
		nav_msgs::Odometry visualLocationMsg;
		ros::Publisher visualLocationPublish;

		/*** flag to indicate that the localization has been initialized
			which means it has seen at least one tag */
		bool localized;
		/*** flag to tell if I should use a tag sighting to update the location
			this is because we may not want to localize while moving since it gives 
			large orientation errors */
		bool updateLocalization;

		// timer for real time loop
		ros::Timer loopTimer;
		void loopTimerHandler(const ros::TimerEvent& event);

		// client for commands on service
		ros::ServiceClient landmarksClient;

		tf::TransformListener listener;
		// store the last transform from map to odom frames
		tf::Transform tParentToChild;
    tf::TransformBroadcaster tf_pub;
		// frame to localize to (usually map or world)
		std::string parent_frame;
		// frame to localize (usually odom or camera)
		std::string child_frame;
};
#endif
