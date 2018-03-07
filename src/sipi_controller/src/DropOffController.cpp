#include "sipi_controller/DropOffController.h"
#include <iostream>
#include <string>
#include "sipi_controller/targetFunctions.h"
#define NUM_ALLOWED_MISSES 10
using namespace Dropoff; 

DropOffController::DropOffController() {
	result.state = result.nextState = State::IDLE;
}

void DropOffController::reset(void) {
	result.state = result.nextState = State::IDLE;
	result.result = ResultCode::SUCCESS;
	missedTargetCount = 0;
	targetLost = true;
	stateStartTime= ros::Time::now();
}
geometry_msgs::Pose2D getAveragePose(
		const std::vector<geometry_msgs::Pose2D>& home_tags)
{
	geometry_msgs::Pose2D avg;
	if (home_tags.empty()) return avg;
	for ( auto &t : home_tags) {
		avg.x += t.x;
		avg.y += t.y;
		avg.theta += t.theta;
	}
	float n = home_tags.size();
	avg.x /= n;
	avg.y /= n;
	avg.theta /= n;
	return avg;
}

Result DropOffController::execute(
		const std::vector<geometry_msgs::Pose2D> &home_tags) 
{
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

	// this checks if it can see a target and gives its position
	bool targetVisible =  !home_tags.empty();
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

	geometry_msgs::Pose2D avg_tag = getAveragePose(home_tags); 
	// state machine switch
	switch(result.state) {
		case State::IDLE:
			if(stateRunTime > ros::Duration(1.0)) {
				if(targetLost) {
					result.result = ResultCode::FAIL;
				} else {
					result.nextState = State::CENTER;
				}
			}
			break;
		case State::CENTER:
			// turn till average of  tags is centered
			if (targetLost) {
				result.nextState = State::IDLE;
				result.result = ResultCode::FAIL;
			}		
			if (avg_tag.y > 0.01) result.cmd_vel.angular.z = 0.1;
			else if (avg_tag.y < -0.01) result.cmd_vel.angular.z = -0.1;
			if(stateRunTime > ros::Duration(3.0)) {
				result.nextState = State::FORWARDSTEER;
			}
			break;
		case State::FORWARDSTEER:
			result.cmd_vel.linear.x = 0.1;
			result.cmd_vel.angular.z = limit(-avg_tag.theta, 0.2);
			if(home_tags.empty() || stateRunTime > ros::Duration(3.0)) {
				result.nextState = State::FORWARD;
			}
			break;
		case State::FORWARD:
			// forward for x seconds to get to clear area in center
			result.cmd_vel.linear.x = 0.1;
			if(stateRunTime >= ros::Duration(2.0)) {
				result.nextState = State::DROP_CUBE;
			}
			break;
		case State::DROP_CUBE:
			// open gripper, raise wrist, wait small delay for fingers to open
			result.grip.fingersOpen = true;
			if(stateRunTime >= ros::Duration(2.0)) {
				result.nextState = State::BACKUP;
			}
			break;
		case State::BACKUP:
			// backward for x seconds to get clear of base before continuing
			result.grip.fingersOpen = true;
			result.cmd_vel.linear.x = -0.1;
			if(stateRunTime >= ros::Duration(3.0)) {
				result.nextState = State::IDLE;
				result.result = ResultCode::SUCCESS;
			}
			break;
	}
	std::ostringstream ss;
	ss << "Dropoff State: "<<(int)result.state <<
		" stateRunTime= "<<std::setprecision(1) << stateRunTime<<
		" Avg: " << avg_tag.x << "," << avg_tag.y << "," << avg_tag.theta << ")" <<
		" cmd: ("<<result.cmd_vel.linear.x << ","<<result.cmd_vel.angular.z<<")";
	for( auto t : home_tags) 
		ss << "("<<t.x<<","<<t.y<<","<<t.theta<<")" ;
	result.status = ss.str();
	return result;
}

