#ifndef AVOID_HOME_H
#define AVOID_HOME_H
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <random_numbers/random_numbers.h>
#include "DrivingController.h"

typedef enum {
	AVOID_RESULT_SUCCESS = 0,
	AVOID_RESULT_BUSY,
	AVOID_RESULT_FAILED
} EAvoidResult;

typedef enum {
	AVOID_STATE_IDLE,	// starting point
	AVOID_STATE_PAUSE,	
	AVOID_STATE_RIGHT,	
	AVOID_STATE_FORWARD,
} EAvoidState;

struct AvoidResult {
  geometry_msgs::Twist cmd_vel;
	EAvoidState state;
	EAvoidState nextState;
	EAvoidResult result;
	std::string status;
	int count;
};

class AvoidController {

	public:
		AvoidController();
		void reset(void);
		AvoidResult result;
		AvoidResult execute(bool homeVisible);
	private:
		int count;
		ros::Time stateStartTime; // start time for states
		ros::Duration stateRunTime; // time since last state change
};

#endif 
