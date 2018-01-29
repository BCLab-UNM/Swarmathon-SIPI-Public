#include "sipi_controller/targetFunctions.h"
#define CAMERA_X_OFFSET 0.02
bool checkForTarget(
		const apriltags_ros::AprilTagDetectionArray& targets,
		int tagId
		)
{
	for (int i = 0; i < targets.detections.size(); i++) {
		if (targets.detections[i].id == tagId) {
			return true;
		}
	}
	return false;
}
int countTargets(
		const apriltags_ros::AprilTagDetectionArray& targets,
		int tagId
		)
{
	int count = 0;
	for (int i = 0; i < targets.detections.size(); i++) {
		if (targets.detections[i].id == tagId) {
			count++;
		}
	}
	return count;
}
bool selectNearestTarget(
		const apriltags_ros::AprilTagDetectionArray& targets,
		double& blockDist, double& blockYawError, int tagId
		)
{
	// TODO could have problem if 2 blocks at same distance
	geometry_msgs::Pose tagPose;
	int n = targets.detections.size();
	double minDist = std::numeric_limits<double>::max();
	int closestTarget  = -1;
	//this loop selects the closest visible block 
	geometry_msgs::Pose p;
	for (int i = 0; i < n ; i++) 
	{
		if(targets.detections[i].id == tagId) {
			p = targets.detections[i].pose.pose;
			if (p.position.z < minDist)
			{
				minDist = p.position.z;
				closestTarget = i;
			}
		}
	}
	// if still -1 that means none of the tagId were found
	if(closestTarget == -1) {
		blockDist = blockYawError = 0.0;
		return false;
	}
	tagPose = targets.detections[closestTarget].pose.pose;
	//distance from bottom center of chassis ignoring height.
  blockDist = hypot(tagPose.position.z, tagPose.position.y); 
#if 0
	ROS_INFO_STREAM("tagPose ("<<tagPose.position.x<<","<<tagPose.position.y<<","<<tagPose.position.z<<")" <<
		" blockDist = " << blockDist);
#endif
	if (0 < (blockDist*blockDist - 0.195*0.195))
	{
		blockDist = sqrt(blockDist*blockDist - 0.195*0.195);
		blockYawError = atan((tagPose.position.x + 0.020)/blockDist)*1.05; //angle to block from bottom center of chassis on the horizontal.
	} else {
		blockDist = 0;
		blockYawError = 0;
	}
	if ( blockYawError > 10) blockYawError = 10; //limits block angle error to prevent overspeed from PID.
	if ( blockYawError < - 10) blockYawError = -10; //due to detetionropping out when moveing quickly
	return true;
}

STagInfo countTags(
		const apriltags_ros::AprilTagDetectionArray& targets,
		int tagId
		)
{
	STagInfo info;
	info.homeCount = info.leftCount = info.rightCount = 0;
	geometry_msgs::Point point;
	info.distance = 100000.0;
	double dist;
	//set flags
	for (int i = 0; i < targets.detections.size(); i++) {
		if (targets.detections[i].id == tagId) {
			info.homeCount++;	
			point = targets.detections[i].pose.pose.position;
			if(point.x + CAMERA_X_OFFSET > 0) {
				info.rightCount++;
			} else {
				info.leftCount++;
			}
			dist = hypot(hypot(point.x, point.y), point.z);
			if(dist <  info.distance) {
				info.distance = dist;
			}
		}
	}
	/*
		 if(info.homeCount > 0) {
		 info.averageDist /= (float)info.homeCount;
		 }
	 */
	return info;
}

