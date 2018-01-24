
#include "Localization.h"
using namespace std;
Localization::Localization(std::string _name, ros::NodeHandle _nh)
{
	mNH = _nh;
	name = _name;
	tfListener = new tf::TransformListener();
	odometrySubscriber = mNH.subscribe((name + "/odom/filtered"), 10, 
			&Localization::odometryHandler, this);
	status_publisher = mNH.advertise<std_msgs::String>((name + "/loc_status"), 1, true);
	arenaFrame = name+"/arena";
	odomFrame = name+"/odom";
	publish_status_timer = mNH.createTimer(ros::Duration(1), 
			&Localization::publishStatusTimerEventHandler, this);
}

void Localization::publishStatusTimerEventHandler(const ros::TimerEvent&) {
	std_msgs::String msg;
	ostringstream ss;
	ss << fixed << setprecision(2) << 
		"Arena: ("<<poseArena.x<<","<<poseArena.y<<","<<poseArena.theta<<
		") odom: ("<<poseOdom.x<<","<<poseOdom.y<<","<<poseOdom.theta<< ")"<<
		statusMsg;
	msg.data = ss.str();
	status_publisher.publish(msg);
	statusMsg.clear();
}
// convert a pose in the arena frame to a pose in the odom frame
geometry_msgs::Pose2D Localization::poseOdomToArena( 
		geometry_msgs::Pose2D poseOdom2D
		)
{
	return transformPose(poseOdom2D, odomFrame, arenaFrame);
}
geometry_msgs::Pose2D Localization::getPoseOdom(void) 
{
	return poseOdom;
}
geometry_msgs::Pose2D Localization::getPoseArena(void) 
{
	poseArena = poseOdomToArena(poseOdom);
	return poseArena;
}
// convert a pose in the arena frame to a pose in the odom frame
geometry_msgs::Pose2D Localization::poseArenaToOdom( 
		geometry_msgs::Pose2D poseArena2D
		)
{
	return transformPose(poseArena2D, arenaFrame, odomFrame);
}
// transform a 2D pose from one frame to another
geometry_msgs::Pose2D Localization::transformPose( 
		geometry_msgs::Pose2D poseIn2D, std::string from_frame, std::string to_frame
		)
{
	geometry_msgs::PoseStamped poseIn3D = pose2Dto3D(poseIn2D,from_frame);
	geometry_msgs::PoseStamped poseOut;

	try { 
		tfListener->transformPose(to_frame, ros::Time(0), poseIn3D, from_frame, poseOut);
		return pose3Dto2D(poseOut);
	} catch(tf::TransformException& ex) {
		statusMsg = "Transform Failed from " + from_frame + " to " + to_frame;
		ROS_WARN("Error transform point from %s to %s: %s", 
				from_frame.c_str(), to_frame.c_str(), ex.what());
		geometry_msgs::Pose2D zeroPose;
		zeroPose.x = zeroPose.y = zeroPose.theta = 0.0;
		return zeroPose;
	}
}

geometry_msgs::PoseStamped pose2Dto3D( 
		geometry_msgs::Pose2D &pose2D,
		std::string frame_id
		)
{
	// fill poseArena from pose2D
	geometry_msgs::PoseStamped pose3D;
	pose3D.header.stamp = ros::Time::now();
	pose3D.header.frame_id = frame_id;
	pose3D.pose.orientation = tf::createQuaternionMsgFromYaw(pose2D.theta);
	pose3D.pose.position.x = pose2D.x;
	pose3D.pose.position.y = pose2D.y;
	return pose3D;
}
geometry_msgs::Pose2D pose3Dto2D( 
		geometry_msgs::PoseStamped &pose3D
		)
{
	geometry_msgs::Pose2D pose2D; 
	pose2D.x = pose3D.pose.position.x;
	pose2D.y = pose3D.pose.position.y;
	pose2D.theta = thetaFromQuat(pose3D.pose.orientation);
	return pose2D;
}
//Get theta rotation by converting quaternion orientation to pitch/roll/yaw
double thetaFromQuat(geometry_msgs::Quaternion quat) 
{
	tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	return yaw;
}
void Localization::odometryHandler(const nav_msgs::Odometry::ConstPtr& message) {
	//Get (x,y) location directly from pose
	poseOdom.x = message->pose.pose.position.x;
	poseOdom.y = message->pose.pose.position.y;

	//Get theta rotation by converting quaternion orientation to pitch/roll/yaw
	poseOdom.theta = thetaFromQuat(message->pose.pose.orientation);
//TODO trying to find bottleneck	poseArena = poseOdomToArena(poseOdom);
}

