#include "sipi_controller/AvoidHome.h"
#define PAUSE_TIME 1
#define TURN_RIGHT_TIME 2
#define FORWARD_TIME 3
#define TURN_VEL 0.3
#define FORWARD_VEL 0.2

using namespace AvoidHome;

Controller::Controller() {
	result.state result.next_state = State::IDLE;
	//	reset();
}
void Controller::reset(void)
{
	count = 0;
	result.state result.next_state = State::IDLE;
	stateStartTime =  ros::Time::now();
}

Result Controller::execute(bool homeVisible) {
	// time since last state change
	if(result.nextState != result.state) {
		stateStartTime =  ros::Time::now();
		result.state = result.nextState;
	}
	stateRunTime = ros::Time::now() - stateStartTime;

	result.cmd_vel.angular.z = result.cmd_vel.linear.x = 0.0;
	result.result = ResultCode::BUSY;
	switch(result.state) {
		case State::IDLE:
			count = 0;
			result.nextState = State::PAUSE;
			break;
		case State::PAUSE:
			if(stateRunTime > ros::Duration(PAUSE_TIME)) {
				if(homeVisible) {
					result.nextState = State::RIGHT;
				} else {
					result.result = ResultCode::SUCCESS;
					result.nextState = State::IDLE;
				}
			}
			break;
		case State::RIGHT:
			result.cmd_vel.angular.z = -TURN_VEL;
			if(stateRunTime > ros::Duration(TURN_RIGHT_TIME)) {
				result.nextState = State::FORWARD;
			}
			break;
		case State::FORWARD:
			result.cmd_vel.linear.x = FORWARD_VEL;
			if(stateRunTime > ros::Duration(FORWARD_TIME)) {
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
	result.status = ss.str();
	return result;
}

