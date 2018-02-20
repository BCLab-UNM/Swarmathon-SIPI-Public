#ifndef FIND_HOME_CONTROLLER_H
#define FIND_HOME_CONTROLLER_H
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include "DrivingController.h"

typedef enum {
	LOSER_RESULT_SUCCESS = 0,
	LOSER_RESULT_BUSY,
	LOSER_RESULT_FAILED
} Find_Home_Result;

typedef enum {
	LOSER_STATE_IDLE,	
	LOSER_STATE_GOHOME,
	LOSER_STATE_SEARCH,	
	LOSER_STATE_CONTINGENCY	
} Finding_Home_State;

struct Finding_Home_Result {
	geometry_msgs::Twist cmd_vel;

	Finding_Home_State state;
	Find_Home_Result result;
	std::string status;
};
class FindHomeController
{
 public:
  FindHomeController();
	void reset(void);
	Finding_Home_Result execute(
		int obstacleDetected, 
		bool homeVisible);
  ros::Time  startTime;
	ros::Duration dt;
	Finding_Home_Result result;
	float incrementingYawError;
 
};
#endif // end header define
