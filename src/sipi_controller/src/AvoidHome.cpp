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
float nearestTagDistance(const std::vector<geometry_msgs::Pose2D> &home_tags) {
  double min_dist_sq = std::numeric_limits<double>::max();
  //this loop selects the closest visible block 
  for ( auto &t : home_tags) {
    float d_sq = t.x*t.x + t.y*t.y;
    if (d_sq < min_dist_sq) {
      min_dist_sq = d_sq;
    }
  }
  return sqrt(min_dist_sq);
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
			if(stateRunTime > ros::Duration(1.0)) {
				if(home_tags.empty()) {
					result.result = ResultCode::SUCCESS;
					result.next_state_ = State::IDLE;
				} else {
					result.next_state_ = State::BACKUP;
				}
			}
			break;
		case State::BACKUP:
        result.cmd_vel.linear.x = -0.1;
				if(home_tags.empty()) {
					result.next_state_ = State::RIGHT;
				}
		case State::RIGHT:
			result.cmd_vel.angular.z = -0.3;
			if(stateRunTime > ros::Duration(3.0)) {
				result.next_state_ = State::FORWARD;
			}
			break;
		case State::FORWARD:
			result.cmd_vel.linear.x = 0.1;
			if (!home_tags.empty()) {
					result.next_state_ = State::RIGHT;
			}	
			if(stateRunTime > ros::Duration(4.0)) {
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

