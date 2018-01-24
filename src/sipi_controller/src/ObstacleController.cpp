#include "ObstacleController.h"
#define PAUSE_TIME 1
#define TURN_RIGHT_TIME 2
#define TURN_LEFT_TIME 2
#define FORWARD_TIME 2
#define TURN_AROUND_TIME 3
#define CENTER_FORWARD_TIME 5
#define TURN_VEL 0.3
#define FORWARD_VEL 0.2
#define TURNAROUND 4


ObstacleController::ObstacleController() {
	//	reset();
}
void ObstacleController::reset(void)
{
	count = 0;
	result.state = OBS_STATE_IDLE;
	stateStartTime =  ros::Time::now();
}
/**
 * The obstacle avoidance state machine
 */
ObstacleResult ObstacleController::execute(int obstacleDetected) {
	// time since last state change
	if(result.nextState != result.state) {
		stateStartTime =  ros::Time::now();
		result.state = result.nextState;
	}
	stateRunTime = ros::Time::now() - stateStartTime;

	result.vel.yawError = result.vel.linear = 0.0;
	result.result = OBS_RESULT_BUSY;
	switch(result.state) {
		case OBS_STATE_IDLE:
			count = 0;
			result.nextState = OBS_STATE_PAUSE;
			break;
		case OBS_STATE_PAUSE:
			if(stateRunTime > ros::Duration(PAUSE_TIME)) {
				if(obstacleDetected > 0) {
					result.nextState = OBS_STATE_RIGHT;
				} else {
					result.result = OBS_RESULT_SUCCESS;
					result.nextState = OBS_STATE_IDLE;
				}
			}
			break;
		case OBS_STATE_RIGHT:
			result.vel.yawError = -TURN_VEL;
			if(stateRunTime > ros::Duration(TURN_RIGHT_TIME)) {
				result.nextState = OBS_STATE_FORWARD;
			}
			break;
		case OBS_STATE_FORWARD:
			result.vel.linear = FORWARD_VEL;
			if(stateRunTime > ros::Duration(FORWARD_TIME)) {
				result.nextState = OBS_STATE_LEFT;
			}
			break;
		case OBS_STATE_LEFT:
			result.vel.yawError = TURN_VEL;
			if(stateRunTime > ros::Duration(TURN_LEFT_TIME)) {
				if (obstacleDetected > 0) {
					count++;
					if(count > 3) {
						result.nextState = OBS_STATE_TURNAROUND;
					} else {
						result.nextState = OBS_STATE_RIGHT;
					}
				} else {
					result.result = OBS_RESULT_SUCCESS;
					result.nextState = OBS_STATE_IDLE;
				}
			}
		case OBS_STATE_TURNAROUND:
			result.vel.yawError = TURN_VEL;
			if(stateRunTime > ros::Duration(TURN_AROUND_TIME)) {
				result.nextState = OBS_STATE_CENTER;
			}
			break;
		case OBS_STATE_CENTER:
			result.vel.linear = FORWARD_VEL;
			if(stateRunTime > ros::Duration(CENTER_FORWARD_TIME)) {
				result.result = OBS_RESULT_FAILED;
				result.nextState = OBS_STATE_IDLE;
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
		std::setprecision(2) <<
		" vel="<<result.vel.linear<<","<<result.vel.yawError;
	result.status = ss.str();
	return result;
}

