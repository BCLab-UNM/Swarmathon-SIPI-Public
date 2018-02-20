#include "sipi_controller/AvoidHome.h"
#define PAUSE_TIME 1
#define TURN_RIGHT_TIME 2
#define FORWARD_TIME 3
#define TURN_VEL 0.3
#define FORWARD_VEL 0.2

AvoidController::AvoidController() {
	//	reset();
}
void AvoidController::reset(void)
{
	count = 0;
	result.state = AVOID_STATE_IDLE;
	stateStartTime =  ros::Time::now();
}

AvoidResult AvoidController::execute(bool homeVisible) {
	// time since last state change
	if(result.nextState != result.state) {
		stateStartTime =  ros::Time::now();
		result.state = result.nextState;
	}
	stateRunTime = ros::Time::now() - stateStartTime;

	result.cmd_vel.angular.z = result.cmd_vel.linear.x = 0.0;
	result.result = AVOID_RESULT_BUSY;
	switch(result.state) {
		case AVOID_STATE_IDLE:
			count = 0;
			result.nextState = AVOID_STATE_PAUSE;
			break;
		case AVOID_STATE_PAUSE:
			if(stateRunTime > ros::Duration(PAUSE_TIME)) {
				if(homeVisible) {
					result.nextState = AVOID_STATE_RIGHT;
				} else {
					result.result = AVOID_RESULT_SUCCESS;
					result.nextState = AVOID_STATE_IDLE;
				}
			}
			break;
		case AVOID_STATE_RIGHT:
			result.cmd_vel.angular.z = -TURN_VEL;
			if(stateRunTime > ros::Duration(TURN_RIGHT_TIME)) {
				result.nextState = AVOID_STATE_FORWARD;
			}
			break;
		case AVOID_STATE_FORWARD:
			result.cmd_vel.linear.x = FORWARD_VEL;
			if(stateRunTime > ros::Duration(FORWARD_TIME)) {
				result.result = AVOID_RESULT_SUCCESS;
				result.nextState = AVOID_STATE_IDLE;
			}
			break;
	}
	// create a status string
	std::ostringstream ss;
	ss << " state: "<< result.state <<
		std::setprecision(1) <<" time: " << stateRunTime <<
		" nextState "<< result.nextState  <<
		" result= " << result.result <<
		std::setprecision(2) <<
		" vel="<<result.cmd_vel.linear.x<<","<<result.cmd_vel.angular.z;
	result.status = ss.str();
	return result;
}

