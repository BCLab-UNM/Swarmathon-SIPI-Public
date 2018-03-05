#ifndef LOCALIZATION_H
#define LOCALIZATION_H
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/Pose2D.h>
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
		void odomGPSHandler(const nav_msgs::Odometry::ConstPtr& message);
		geometry_msgs::Pose2D poseArenaToOdom( 
				geometry_msgs::Pose2D poseArena2D
				);
		geometry_msgs::Pose2D poseOdomToArena( 
				geometry_msgs::Pose2D poseOdom2D
				);

    void execute(const std::vector<geometry_msgs::Pose2D> &home_tags);
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
		std::string arenaFrame, mapFrame;
		ros::Publisher status_publisher;
		ros::Subscriber odometrySubscriber;
		ros::Subscriber odomGPSSubscriber;
		geometry_msgs::Pose2D poseOdom, poseGPS, poseArena; 
		geometry_msgs::Pose2D arena_offset; 
    geometry_msgs::Pose2D pose_visual;
		tf::TransformListener *tfListener;
		std::string statusMsg;
    // store the last transform from map to arena frames
    tf::Transform tParentToChild;
    tf::TransformBroadcaster tf_pub;


};

#endif
