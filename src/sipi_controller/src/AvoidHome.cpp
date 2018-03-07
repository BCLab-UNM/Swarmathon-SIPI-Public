#include "sipi_controller/AvoidHome.h"
#define PAUSE_TIME 1
#define TURN_RIGHT_TIME 2
#define FORWARD_TIME 3
#define TURN_VEL 0.3
#define FORWARD_VEL 0.2

using namespace AvoidHome;

Controller::Controller() {
	result.state  = result.next_state_ = State::IDLE;
	//	reset();
}
void Controller::reset(void)
{
	count = 0;
	result.state = result.next_state_ = State::IDLE;
	stateStartTime =  ros::Time::now();
}

Result Controller::execute(const std::vector<geometry_msgs::Pose2D> &home_tags) {
	// time since last state change
	if(result.next_state_ != result.state) {
		stateStartTime =  ros::Time::now();
		result.state = result.next_state_;
	}
	stateRunTime = ros::Time::now() - stateStartTime;

	result.cmd_vel.angular.z = result.cmd_vel.linear.x = 0.0;
	result.result = ResultCode::BUSY;
	switch(result.state) {
		case State::IDLE:
			count = 0;
			result.next_state_ = State::PAUSE;
			break;
		case State::PAUSE:
			if(stateRunTime > ros::Duration(PAUSE_TIME)) {
				if(home_tags.empty()) {
					result.next_state_ = State::RIGHT;
				} else {
					result.result = ResultCode::SUCCESS;
					result.next_state_ = State::IDLE;
				}
			}
			break;
		case State::RIGHT:
			result.cmd_vel.angular.z = -TURN_VEL;
			if(stateRunTime > ros::Duration(TURN_RIGHT_TIME)) {
				result.next_state_ = State::FORWARD;
			}
			break;
		case State::FORWARD:
			result.cmd_vel.linear.x = FORWARD_VEL;
			if(stateRunTime > ros::Duration(FORWARD_TIME)) {
				result.result = ResultCode::SUCCESS;
				result.next_state_ = State::IDLE;
			}
			break;
	}
	// create a status string
	std::ostringstream ss;
	ss << " state: "<< (int)result.state <<
		std::setprecision(1) <<" time: " << stateRunTime <<
		" nextState "<< (int)result.next_state_  <<
		" result= " << (int)result.result <<
		std::setprecision(2) <<
		" vel="<<result.cmd_vel.linear.x<<","<<result.cmd_vel.angular.z;
	result.status = ss.str();
	return result;
}

