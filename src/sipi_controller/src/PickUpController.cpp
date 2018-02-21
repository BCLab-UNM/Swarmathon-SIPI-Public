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
#define PICKUP_FORWARD_VEL 0.2		// how fast to go forward to scoop
#define PICKUP_BACKUP_VEL 0.2	// how fast to go backward to try to reacquire
#define PICKUP_TURN_CONSTANT 0.2		// how fast to turn open loop
using namespace PickupController; 
PickUpController::PickUpController(void) 
{
  //	reset();
}

void PickUpController::reset() {
  result.cmd_vel.linear.x = 0;
  result.cmd_vel.angular.z = 0;
  result.grip.fingersOpen = true;
  result.grip.wristPos = WRIST_UP;
  result.state = PickupController::State::IDLE;
  result.nextState = PickupController::State::IDLE;
  stateStartTime = ros::Time::now();
  targetLost = true;
}
Result PickUpController::execute(
    int obstacleDetected,
    const apriltags_ros::AprilTagDetectionArray& targets
    ) 
{
  // default is to send no velocity and be BUSY
  result.cmd_vel.linear.x = result.cmd_vel.angular.z = 0.0;
  result.result = ResultCode::BUSY;
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
    case PickupController::State::IDLE:
      result.grip.fingersOpen = true;
      result.grip.wristPos = WRIST_UP;
      if(stateRunTime > ros::Duration(1.0)) {
        if(targetLost) {
          ROS_WARN("PICKUP target lost from IDLE!");
          result.nextState = PickupController::State::BACKUP;
        } else {
          result.nextState = PickupController::State::CENTER_COARSE;
        }
      }
      break;
    case PickupController::State::CENTER_COARSE:
      // turn until selected block is centered in front
      // allow 5 sec then proceed if cube is still visible
      result.grip.fingersOpen = true;
      result.grip.wristPos = WRIST_DOWN;
      result.cmd_vel.angular.z = blockYawError > 0 ? -0.2 : 0.2;
//      result.cmd_vel.linear.x = distErr > 0 ? 0.05 : 0.05;
      if(stateRunTime > ros::Duration(3.0)) {
        if(targetLost) {
          ROS_WARN("PICKUP target lost!");
          result.nextState = PickupController::State::BACKUP;
        } else {
          result.nextState = PickupController::State::DIST;
        }
      }
      break;
    case PickupController::State::DIST:
      // move  to right distance
      result.grip.fingersOpen = true;
      result.grip.wristPos = WRIST_DOWN;
 //     result.cmd_vel.angular.z = limit(-blockYawError, 0.2);
      result.cmd_vel.linear.x = limit(distErr, 0.2);
      if(stateRunTime > ros::Duration(3)) {
        if(targetLost) {
          result.nextState = PickupController::State::BACKUP;
        } else {
          result.nextState = PickupController::State::FORWARD;
        }
      }
      break;
    case PickupController::State::FORWARD:
      // drive forward until block should be in gripper
      result.grip.fingersOpen = true;
      result.grip.wristPos = WRIST_DOWN;
      result.cmd_vel.linear.x = PICKUP_FORWARD_VEL;
      if(stateRunTime >= ros::Duration(PICKUP_FORWARD_TIME)) {
        result.nextState = PickupController::State::PICKUP;
      }
      break;
    case PickupController::State::PICKUP:
      result.grip.fingersOpen = false;
      result.grip.wristPos = WRIST_DOWN;
      if(stateRunTime >= ros::Duration(PICKUP_PICKUP_TIME)) {
        result.nextState = PickupController::State::VERIFY;
      }
      break;
    case PickupController::State::VERIFY:
      result.grip.fingersOpen = false;
      result.grip.wristPos = WRIST_VERIFY;
      if(obstacleDetected == 4 || (blockDist < 0.1 && targetVisible)) {
        result.result = ResultCode::SUCCESS;
        result.nextState = PickupController::State::IDLE;
      } else if(stateRunTime > ros::Duration(PICKUP_VERIFY_TIME)) {
        result.nextState = PickupController::State::BACKUP;
      }
      break;
    case PickupController::State::BACKUP:
      result.grip.fingersOpen = true;
      result.grip.wristPos = WRIST_UP;
      result.cmd_vel.linear.x = -PICKUP_BACKUP_VEL;
      if(stateRunTime >= ros::Duration(PICKUP_BACKUP_TIME)) {
        // if can still see target, try again
        if(targetLost) {
          result.result = ResultCode::FAILED;
          result.nextState = PickupController::State::IDLE;
        } else {
          result.nextState = PickupController::State::IDLE;
        }
      }
      break;
  }
  // create a status string
  std::ostringstream ss;
  ss << " state: " << (int)result.state <<
    std::setprecision(1) <<" time: " << stateRunTime <<
    " nextState "<< (int)result.nextState  <<
    " result= " << (int)result.result <<
    " obstacle= "<< obstacleDetected <<
    " visible= "<< targetVisible <<
    std::setprecision(2) <<
    " cube pos= "<<blockDist<<","<<blockYawError << 
    ") missed="<<missedTargetCount <<
    " distErr="<<distErr<<
    " vel="<<result.cmd_vel.linear.x<<","<<result.cmd_vel.angular.z;
  result.status = ss.str();
  return result;
}

PickUpController::~PickUpController() {
}
