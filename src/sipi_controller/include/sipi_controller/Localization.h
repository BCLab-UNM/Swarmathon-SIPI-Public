#ifndef LOCALIZATION_H
#define LOCALIZATION_H
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
// helper functions to work with poses and angles
// get 2D theta from quaternion
double thetaFromQuat(geometry_msgs::Quaternion quat);
// get 2D pose from 3D
geometry_msgs::Pose2D pose3Dto2D( 
		geometry_msgs::PoseStamped &pose3D
		);
// get 3D stamped pose from 2D
geometry_msgs::PoseStamped pose2Dto3D( 
		geometry_msgs::Pose2D &pose2D,
		std::string frame_id
		);
class Localization 
{
	public:
		Localization(std::string _name, ros::NodeHandle _nh);
		void odometryHandler(const nav_msgs::Odometry::ConstPtr& message);
		geometry_msgs::Pose2D poseArenaToOdom( 
				geometry_msgs::Pose2D poseArena2D
				);
		geometry_msgs::Pose2D poseOdomToArena( 
				geometry_msgs::Pose2D poseOdom2D
				);

		geometry_msgs::Pose2D getPoseOdom(void); 
		geometry_msgs::Pose2D getPoseArena(void); 
		geometry_msgs::Pose2D transformPose( 
				geometry_msgs::Pose2D poseIn2D, // pose to transform
				std::string from_frame,					// frame pose is in 
				std::string to_frame						// frame to transform pose into
				);
	private:
		ros::NodeHandle  mNH;
		std::string name;
		std::string arenaFrame, odomFrame;
		ros::Publisher status_publisher;
		ros::Subscriber odometrySubscriber;
		geometry_msgs::Pose2D poseOdom, poseArena; 
		tf::TransformListener *tfListener;
		ros::Timer publish_status_timer;
		void publishStatusTimerEventHandler(const ros::TimerEvent&);
		std::string statusMsg;
};

#endif
