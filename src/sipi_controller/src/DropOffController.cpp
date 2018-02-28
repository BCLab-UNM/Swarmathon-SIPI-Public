#include "sipi_controller/DropOffController.h"
#include <iostream>
#include <string>
#include "sipi_controller/targetFunctions.h"
#define FORWARD_VEL 0.18					// vel to approach center area
#define APPROACH_TIME 2						// time to approach center
#define BACKUP_TIME 3							// time to back out of center
#define DROP_CUBE_TIME 1  				// time to allow cube to drop
#define CENTERING_DISTANCE 0.2		// how close to average tag location to be
#define NUM_ALLOWED_MISSES 10
#define CENTERING_YAW_ERROR 0.2		// open loop how fast to turn to center
using namespace Dropoff; 

DropOffController::DropOffController() {
  //reset();
}
void DropOffController::reset(void) {
  result.state = result.nextState = State::IDLE;
  result.result = ResultCode::SUCCESS;
  missedTargetCount = 0;
  targetLost = true;
  stateStartTime= ros::Time::now();
}

Result DropOffController::execute(
    const apriltags_ros::AprilTagDetectionArray& targets,
    const std::vector<geometry_msgs::Pose2D> &home_tags
    ) 
{
  STagInfo tagInfo = countTags(targets, HOME_TAG_ID);
  result.grip.fingersOpen = false;
  result.grip.wristPos = WRIST_UP;
  result.cmd_vel.linear.x = result.cmd_vel.angular.z = 0.0;
  result.result = ResultCode::BUSY;
  // time since last state change
  if(result.nextState != result.state) {
    stateStartTime =  ros::Time::now();
    result.state = result.nextState;
  }
  stateRunTime = ros::Time::now() - stateStartTime;

  double blockDist, blockYawError;
  // this checks if it can see a target and gives its position
  bool targetVisible = selectNearestTarget(targets, blockDist, 
      blockYawError, HOME_TAG_ID);
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
  // how far are we from centering point
  float distErr = blockDist - CENTERING_DISTANCE;
  // state machine switch
  switch(result.state) {
    case State::IDLE:
      if(stateRunTime > ros::Duration(1.0)) {
        if(targetLost) {
          result.result = ResultCode::FAIL;
        } else {
          result.nextState = State::FORWARD;
        }
      }
      break;
    case State::CENTER:
      // turn till nearest tag is centered
      //result.cmd_vel.angular.z = limit(-blockYawError, 0.2);
      if(tagInfo.leftCount > tagInfo.rightCount) {
        result.cmd_vel.angular.z = CENTERING_YAW_ERROR;
      } else if(tagInfo.leftCount < tagInfo.rightCount) {
        result.cmd_vel.angular.z = -CENTERING_YAW_ERROR;
      } 
      if(stateRunTime > ros::Duration(3.0)) {
        if(targetLost) {
          result.result = ResultCode::FAIL;
        } else {
          result.nextState = State::FORWARD;
        } 
      }
      break;
    case State::FORWARD:
      // forward for x seconds to get to clear area in center
      result.cmd_vel.linear.x = FORWARD_VEL;
      if(stateRunTime >= ros::Duration(APPROACH_TIME)) {
        result.nextState = State::DROP_CUBE;
      }
      break;
    case State::DROP_CUBE:
      // open gripper, raise wrist, wait small delay for fingers to open
      result.grip.fingersOpen = true;
      if(stateRunTime >= ros::Duration(DROP_CUBE_TIME)) {
        result.nextState = State::BACKUP;
      }
      break;
    case State::BACKUP:
      // backward for x seconds to get clear of base before continuing
      result.grip.fingersOpen = true;
      result.cmd_vel.linear.x = -FORWARD_VEL;
      if(stateRunTime >= ros::Duration(BACKUP_TIME)) {
        result.nextState = State::IDLE;
        result.result = ResultCode::SUCCESS;
      }
      break;
  }
  std::ostringstream ss;
  ss << "Dropoff State: "<<(int)result.state <<
    " stateRunTime= "<<std::setprecision(1) << stateRunTime<<
    " Tags: "<<tagInfo.leftCount<<","<<tagInfo.rightCount<<
    " Dist: "<<tagInfo.distance <<
    " blockDist= "<< blockDist << 
    " blockYawError= " << blockYawError <<
    " cmd: ("<<result.cmd_vel.linear.x << ","<<result.cmd_vel.angular.z<<")";
  for( auto t : home_tags) 
        ss << "("<<t.x<<","<<t.y<<","<<t.theta<<")" ;
  result.status = ss.str();
  return result;
}

