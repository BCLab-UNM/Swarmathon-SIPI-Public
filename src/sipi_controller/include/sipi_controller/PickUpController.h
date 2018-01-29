#ifndef PICKUPCONTROLLER_H
#define PICKUPCONTROLLER_H
#define HEADERFILE_H
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <ros/ros.h>
#include "GripperController.h"
#include "DrivingController.h"
#include "targetFunctions.h"

typedef enum {
	PICKUP_RESULT_SUCCESS = 0,
	PICKUP_RESULT_BUSY,
	PICKUP_RESULT_FAILED
} EPickupResult;

typedef enum {
	PICKUP_STATE_IDLE,	// starting point
	PICKUP_STATE_CENTER_COARSE,	// turn so that cube is in center
	PICKUP_STATE_DIST,	//move to the correct distance
	PICKUP_STATE_FORWARD,	// go forward to scoop up cube
	PICKUP_STATE_PICKUP,		// pick up cube
	PICKUP_STATE_VERIFY,		// check if successful
	PICKUP_STATE_BACKUP		// back up a little to reacquire lost target
} EPickupState;

struct PickUpResult {
	CDriveCmd vel;
	CGripCmd grip;
	bool pickedUp;
	bool giveUp;
	EPickupState state;
	EPickupState nextState;
	int result;
	std::string status;
};
class PickUpController
{
	public:
		PickUpController();
		~PickUpController();

		PickUpResult execute(
				int obstacleDetection,
				const apriltags_ros::AprilTagDetectionArray& targets
				); 

		void reset();

	private:
		//yaw error to target block 
		double blockYawError;
		//distance to target block from front of robot
		double blockDist;
		double distErr;

		//struct for returning data to mobility
		PickUpResult result;

		ros::Time stateStartTime; // start time for states
		ros::Duration stateRunTime; // time since last state change
		geometry_msgs::Pose tagPose;
		int missedTargetCount;
		// flag to indicate that the target is no longer visible
		bool targetLost;
};
#endif // end header define
