#include "sipi_controller/PickUpController.h"
#include <iostream>
#include <iostream>
#include "sipi_controller/GripperController.h"
// offset to center of screen in X
#define X_OFFSET 0.054
#define X_TOLERANCE 0.02
//threshold distance to be from the target block before attempting pickup
#define DIST_TOLERANCE 0.05
// how far away should the cube be before just going forward
#define PICKUP_DISTANCE 0.2 
// how many times can you not see a target before deciding it is out of view
// this is to prevent state changes because of blurs
#define NUM_ALLOWED_MISSES 10
// timing constants
#define PICKUP_CENTER_TIMEOUT 2.0 // how long to try to center target
#define PICKUP_FORWARD_TIME 1.5		// how long to drive forward to scoop cube
#define PICKUP_PICKUP_TIME 1.0		// how to wait for pickup to close
#define PICKUP_BACKUP_TIME 1.0		// how long to backup after done
#define PICKUP_VERIFY_TIME 1.0		// how long to try to verify target
#define PICKUP_FORWARD_VEL 0.1		// how fast to go forward to scoop
#define PICKUP_BACKUP_VEL 0.18	// how fast to go backward to try to reacquire
#define PICKUP_TURN_CONSTANT 0.15		// how fast to turn open loop
PickUpController::PickUpController(void) 
{
//	reset();
}
void PickUpController::reset() {
	result.vel.linear = 0;
	result.vel.yawError = 0;
	result.grip.fingersOpen = true;
	result.grip.wristPos = WRIST_UP;
	result.state = PICKUP_STATE_IDLE;
	result.nextState = PICKUP_STATE_IDLE;
	stateStartTime = ros::Time::now();
	targetLost = true;
}
PickUpResult PickUpController::execute(
		int obstacleDetected,
		const apriltags_ros::AprilTagDetectionArray& targets
		) 
{
	// default is to send no velocity and be BUSY
	result.vel.linear = result.vel.yawError = 0.0;
	result.result = PICKUP_RESULT_BUSY;
	// time since last state change
	if(result.nextState != result.state) {
		stateStartTime =  ros::Time::now();
		result.state = result.nextState;
	}
	stateRunTime = ros::Time::now() - stateStartTime;
	// this checks if it can see a target and gives its position
	bool targetVisible = selectNearestTarget(targets, blockDist, 
			blockYawError, BLOCK_TAG_ID);
	// this allows the target to dissappear briefly but not consider it lost
	// at first (reset) targetLost is true. but then it only becomes true
	// again if the target is missed multiple times
	if(targetVisible) {
		missedTargetCount = 0;
		targetLost = false;
	} else {
		missedTargetCount++;
		if(missedTargetCount > NUM_ALLOWED_MISSES) {
			targetLost = true;
		}
	}
	// how far are we from the drive forward point
	distErr = blockDist - PICKUP_DISTANCE;
	switch(result.state) {
		case PICKUP_STATE_IDLE:
			result.grip.fingersOpen = true;
			result.grip.wristPos = WRIST_UP;
			if(!targetLost) {
				result.nextState = PICKUP_STATE_CENTER_COARSE;
			} else {
				result.nextState = PICKUP_STATE_BACKUP;
			}
			break;
		case PICKUP_STATE_CENTER_COARSE:
			// turn until selected block is centered in front
			// allow 5 sec then proceed if cube is still visible
			result.grip.fingersOpen = true;
			result.grip.wristPos = WRIST_DOWN;
			result.vel.yawError = limit(-blockYawError, 0.2);
			if(stateRunTime > ros::Duration(3)) {
				if(targetLost) {
					result.nextState = PICKUP_STATE_BACKUP;
				} else {
					result.nextState = PICKUP_STATE_DIST;
				}
			}
			break;
		case PICKUP_STATE_DIST:
			// move  to right distance
			// allow 5 sec then proceed
			result.grip.fingersOpen = true;
			result.grip.wristPos = WRIST_DOWN;
			result.vel.yawError = limit(-blockYawError, 0.2);
			result.vel.linear = limit(distErr, 0.2);
			if(stateRunTime > ros::Duration(3)) {
				if(targetLost) {
					result.nextState = PICKUP_STATE_BACKUP;
				} else {
					result.nextState = PICKUP_STATE_FORWARD;
				}
			}
			break;
		case PICKUP_STATE_FORWARD:
			// drive forward until block should be in gripper
			result.grip.fingersOpen = true;
			result.grip.wristPos = WRIST_DOWN;
			result.vel.linear = PICKUP_FORWARD_VEL;
			if(stateRunTime >= ros::Duration(PICKUP_FORWARD_TIME)) {
				result.nextState = PICKUP_STATE_PICKUP;
			}
			break;
		case PICKUP_STATE_PICKUP:
			result.grip.fingersOpen = false;
			result.grip.wristPos = WRIST_DOWN;
			if(stateRunTime >= ros::Duration(PICKUP_PICKUP_TIME)) {
				result.nextState = PICKUP_STATE_VERIFY;
			}
			break;
		case PICKUP_STATE_VERIFY:
			result.grip.fingersOpen = false;
			result.grip.wristPos = WRIST_VERIFY;
			if(obstacleDetected == 4 || (blockDist < 0.1 && targetVisible)) {
				result.result = PICKUP_RESULT_SUCCESS;
				result.nextState = PICKUP_STATE_IDLE;
			} else if(stateRunTime > ros::Duration(PICKUP_VERIFY_TIME)) {
				result.nextState = PICKUP_STATE_BACKUP;
			}
			break;
		case PICKUP_STATE_BACKUP:
			result.grip.fingersOpen = true;
			result.grip.wristPos = WRIST_UP;
			result.vel.linear = -PICKUP_BACKUP_VEL;
			if(stateRunTime >= ros::Duration(PICKUP_BACKUP_TIME)) {
				// if can still see target, try again
				if(targetLost) {
					result.result = PICKUP_RESULT_FAILED;
					result.nextState = PICKUP_STATE_IDLE;
				} else {
					result.nextState = PICKUP_STATE_IDLE;
				}
			}
			break;
	}
	// create a status string
	std::ostringstream ss;
	ss << " state: "<< result.state <<
		std::setprecision(1) <<" time: " << stateRunTime <<
		" nextState "<< result.nextState  <<
		" result= " << result.result <<
		" obstacle= "<< obstacleDetected <<
		" visible= "<< targetVisible <<
		std::setprecision(2) <<
		" cube pos= "<<blockDist<<","<<blockYawError << 
		") missed="<<missedTargetCount <<
		" distErr="<<distErr<<
		" vel="<<result.vel.linear<<","<<result.vel.yawError;
	result.status = ss.str();
	return result;
}


PickUpController::~PickUpController() {
}
