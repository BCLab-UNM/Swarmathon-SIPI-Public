#ifndef DROPOFCONTROLLER_H
#define DROPOFCONTROLLER_H

#include <apriltags_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include "GripperController.h"
#include "DrivingController.h"
typedef enum {
	DROPOFF_RESULT_SUCCESS = 0,
	DROPOFF_RESULT_BUSY,
	DROPOFF_RESULT_FAIL
} EDropoffResult;

typedef enum {
	DROPOFF_STATE_IDLE = 0,
	DROPOFF_STATE_CENTER,
	DROPOFF_STATE_FORWARD,
	DROPOFF_STATE_DROP_CUBE,
	DROPOFF_STATE_BACKUP
} EDropoffState;

struct DropOffResult {
  geometry_msgs::Twist cmd_vel;
	CGripCmd grip;
	EDropoffState state;
	EDropoffState nextState;
	EDropoffResult result;
	std::string status;
};

class DropOffController
{
	public:
		DropOffController(void);
		void reset(void);
		DropOffResult execute(
				const apriltags_ros::AprilTagDetectionArray& targets,
        const std::vector<geometry_msgs::Pose2D> &home_tags
				);
	private:
		DropOffResult result;
		ros::Time stateStartTime; // start time for states
		ros::Duration stateRunTime; // time since last state change
		bool targetLost;
		int missedTargetCount;
};
#endif // end header define
