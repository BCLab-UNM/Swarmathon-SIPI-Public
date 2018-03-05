#ifndef FIND_HOME_CONTROLLER_H
#define FIND_HOME_CONTROLLER_H
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include "DrivingController.h"

namespace FindHome {

enum class ResultCode {
	SUCCESS = 0,
	BUSY,
	FAILED
};

enum class State {
	IDLE,	
	SEARCH
};

struct Result {
	geometry_msgs::Twist cmd_vel;

	State state;
	ResultCode result;
	std::string status;
};

class Controller
{
 public:
  Controller();
	void reset(void);
	Result execute(
		int obstacleDetected, 
		bool homeVisible);
  ros::Time  startTime;
	ros::Duration dt;
	Result result;
};

}
#endif // end header define
