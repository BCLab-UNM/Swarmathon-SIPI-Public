#ifndef TARGET_FUNCTIONS_H
#define TARGET_FUNCTIONS_H
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <ros/ros.h>
/***
	generic functions regarding analysis of april tag targets
 */
#define HOME_TAG_ID 256
#define BLOCK_TAG_ID 0
/***
	find the nearest target in <targets> with id = <tagId>
	calculate its distance and yaw error
	return true if the block is seen, false if not
 */
bool selectNearestTarget(
		const apriltags_ros::AprilTagDetectionArray& targets,
		double& blockDist, double& blockYawError, int tagId
		);
/***
	check if a target with id = <tagId> is visible
 */
bool checkForTarget(
		const apriltags_ros::AprilTagDetectionArray& targets,
		int tagId
		);
/***
	count how many targets with id= <tagId> are visible
 */
int countTargets(
		const apriltags_ros::AprilTagDetectionArray& targets,
		int tagId
		);

typedef struct {
	int homeCount;
	int leftCount;
	int rightCount;
	double distance;
} STagInfo;
STagInfo countTags(
		const apriltags_ros::AprilTagDetectionArray& targets, 
		int tagId
		);
#endif // end header define
