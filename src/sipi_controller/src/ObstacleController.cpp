#include "sipi_controller/ObstacleController.h"
#include <angles/angles.h>

using namespace Obstacle; 

ObstacleController::ObstacleController(void) {
  result.state = result.nextState = State::IDLE;
  direction_ = 1;
}
#define MIN_DISTANCE 0.5
void ObstacleController::reset(void) {
  count = 0;
  result.state = result.nextState = State::IDLE;
  stateStartTime =  ros::Time::now();
}

bool Obstacle::obstacleDetected(const geometry_msgs::Point &ultrasound) {
  double distance = MIN_DISTANCE;
  return (ultrasound.x < distance || ultrasound.y < distance || 
      ultrasound.z < distance);
}
bool obstacleLeft(const geometry_msgs::Point &ultrasound) {
  double distance = MIN_DISTANCE;
  return (ultrasound.x < distance);
}
bool obstacleRight(const geometry_msgs::Point &ultrasound) {
  double distance = MIN_DISTANCE;
  return (ultrasound.z < distance);
}
bool obstacleCenter(const geometry_msgs::Point &ultrasound) {
  double distance = MIN_DISTANCE;
  return (ultrasound.y < distance);
}

/**
 * The obstacle avoidance state machine
 */
Result ObstacleController::execute(
    const geometry_msgs::Point &ultrasound, float heading) {

  int obstacle_detected = obstacleDetected(ultrasound);
  // time since last state change

  if(result.nextState != result.state) {
    stateStartTime =  ros::Time::now();
    result.state = result.nextState;
  }
  stateRunTime = ros::Time::now() - stateStartTime;

  result.cmd_vel.angular.z = result.cmd_vel.linear.x = 0.0;
  result.result = ResultCode::BUSY;
  bool left, right, center;
  left = obstacleLeft(ultrasound);
  right = obstacleRight(ultrasound);
  center = obstacleCenter(ultrasound);
  switch(result.state) {
    case State::IDLE:
      count = 0;
      result.nextState = State::PAUSE;
      break;
    case State::PAUSE:
      if(stateRunTime > ros::Duration(1.0)) {
        initial_heading = heading;
        if(obstacle_detected) {
          result.nextState = State::DECIDE;
        } else {
          result.result = ResultCode::SUCCESS;
          result.nextState = State::IDLE;
        }
      }
      break;

    case State::DECIDE:
      if(obstacle_detected) {
        if(ultrasound.x < ultrasound.z) {
          direction_ = -1.0;
        } else {
          direction_ = 1.0;
        }
        result.nextState = State::TURN_UNITL_CLEAR;
      } else {
        result.result = ResultCode::SUCCESS;
        result.nextState = State::IDLE;
      }
      break;
    case State::TURN_UNITL_CLEAR:
      //      result.cmd_vel.linear.x = 0.2;
      result.cmd_vel.angular.z = 0.6 * direction_;
      if(!obstacle_detected) {
        result.nextState = State::TURN_UNITL_PARALLEL;
      }
      break;
    case State::TURN_UNITL_PARALLEL:
      //      result.cmd_vel.linear.x = 0.2;
      result.cmd_vel.angular.z = 0.6 * direction_;
      if(stateRunTime > ros::Duration(0.8)) {
        result.nextState = State::FORWARD_PARALLEL;
      }
      break;
    case State::FORWARD_PARALLEL:
      result.cmd_vel.linear.x = 0.2;
      if(obstacle_detected) {
        result.nextState = State::TURN_UNITL_CLEAR;
      } else if(stateRunTime > ros::Duration(1.5)) {
        result.nextState = State::TURN_TO_CHECK;
      }
      break;
    case State::TURN_TO_CHECK:
      result.cmd_vel.angular.z = -0.6 * direction_;
      if(obstacle_detected) {
        result.nextState = State::TURN_UNITL_CLEAR;
      } else if(fabs(angles::shortest_angular_distance(initial_heading, 
          heading)) < 0.1) {
        result.result = ResultCode::SUCCESS;
        result.nextState = State::IDLE;
      } else if(stateRunTime > ros::Duration(2.5)) {
        result.nextState = State::FORWARD_CLEAR;
      }
      break;
    case State::FORWARD_CLEAR:
      result.cmd_vel.linear.x = 0.2;
      if(obstacle_detected) {
        result.nextState = State::TURN_UNITL_CLEAR;
      } else if(stateRunTime > ros::Duration(3)) {
        result.result = ResultCode::SUCCESS;
        result.nextState = State::IDLE;
      }
      break;
  }

  // create a status string
  std::ostringstream ss;
  ss << " state: "<< (int)result.state <<
    std::setprecision(1) <<" time: " << stateRunTime <<
    " nextState "<< (int)result.nextState  <<
    " result= " << (int)result.result <<
    std::setprecision(2) <<
    " vel="<<result.cmd_vel.linear.x<<","<<result.cmd_vel.angular.z;
  ss << " OBS "<< obstacle_detected <<","<< count << "("<<ultrasound.x<<","<<ultrasound.y<<","<<ultrasound.z<<")";
  result.status = ss.str();
  return result;
}

