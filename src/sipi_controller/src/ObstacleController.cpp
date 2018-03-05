#include "sipi_controller/ObstacleController.h"

using namespace Obstacle; 

ObstacleController::ObstacleController(void) {
  result.state = result.nextState = State::IDLE;
}

void ObstacleController::reset(void) {
  count = 0;
  result.state = result.nextState = State::IDLE;
  stateStartTime =  ros::Time::now();
}

bool Obstacle::obstacleDetected(const geometry_msgs::Point &ultrasound) {
  double distance = 0.5;
  return (ultrasound.x < distance || ultrasound.y < distance || 
      ultrasound.z < distance);
}
bool obstacleLeft(const geometry_msgs::Point &ultrasound) {
  double distance = 0.5;
  return (ultrasound.x < distance);
}
bool obstacleRight(const geometry_msgs::Point &ultrasound) {
  double distance = 0.5;
  return (ultrasound.z < distance);
}
bool obstacleCenter(const geometry_msgs::Point &ultrasound) {
  double distance = 0.5;
  return (ultrasound.y < distance);
}

/**
 * The obstacle avoidance state machine
 */
Result ObstacleController::execute(
    const geometry_msgs::Point &ultrasound) {

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
          if(left && !center && !right) result.nextState = State::GENTLE_TURN_RIGHT;
          else if(!left && !center && right) result.nextState = State::GENTLE_TURN_LEFT;
          else if(left && center && !right) result.nextState = State::HARD_TURN_RIGHT;
          else if(!left && center && right) result.nextState = State::HARD_TURN_LEFT;
          else result.nextState = State::HARD_TURN_RIGHT;
        } else {
          result.result = ResultCode::SUCCESS;
          result.nextState = State::IDLE;
        }
      break;
    case State::GENTLE_TURN_RIGHT:
//      result.cmd_vel.linear.x = 0.2;
      result.cmd_vel.angular.z = -0.5;
      if(!obstacle_detected) {
        result.nextState = State::FORWARD;
      }
      break;
    case State::GENTLE_TURN_LEFT:
//      result.cmd_vel.linear.x = 0.2;
      result.cmd_vel.angular.z = 0.5;
      if(!obstacle_detected) {
        result.nextState = State::FORWARD;
      }
      break;
    case State::HARD_TURN_RIGHT:
      result.cmd_vel.angular.z = -0.5;
      if(!center) {
        result.nextState = State::DECIDE;
      }
      break;
    case State::HARD_TURN_LEFT:
      result.cmd_vel.angular.z = 0.5;
      if(!center) {
        result.nextState = State::DECIDE;
      }
      break;
    case State::FORWARD:
      result.cmd_vel.linear.x = 0.2;
      if(obstacle_detected) {
        result.nextState = State::DECIDE;
      } else if(stateRunTime > ros::Duration(1)) {
        result.result = ResultCode::SUCCESS;
        result.nextState = State::IDLE;
      }
      break;
    case State::RIGHT2:
      result.cmd_vel.angular.z = -0.5;
      if(stateRunTime > ros::Duration(1)) {
        result.nextState = State::FORWARD;
      }
      break;
    case State::LEFT:
      result.cmd_vel.angular.z = 0.5;
      if(obstacle_detected) {
        result.nextState = State::RIGHT;
      } else if(stateRunTime > ros::Duration(6)) {
        if (obstacle_detected) {
          count++;
          if(count > 3) {
            result.nextState = State::TURNAROUND;
          } else {
            result.nextState = State::RIGHT;
          }
        } else {
          result.result = ResultCode::SUCCESS;
          result.nextState = State::IDLE;
        }
      }
      break;
    case State::TURNAROUND:
      result.cmd_vel.angular.z = 0.3;
      if(stateRunTime > ros::Duration(3.0)) {
        result.nextState = State::CENTER;
      }
      break;
    case State::CENTER:
      result.cmd_vel.linear.x = 0.2;
      if(stateRunTime > ros::Duration(5.0)) {
        result.result = ResultCode::FAILED;
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

