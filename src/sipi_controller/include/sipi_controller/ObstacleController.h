#ifndef Obstacle_CONTROLLER
#define Obstacle_CONTROLLER
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <random_numbers/random_numbers.h>
#include "DrivingController.h"

/**
 * This class implements the search control algorithm for the rovers. The code
 * here should be modified and enhanced to improve search performance.
 */
typedef enum {
	OBS_RESULT_SUCCESS = 0,
	OBS_RESULT_BUSY,
	OBS_RESULT_FAILED
} EObstacleResult;

typedef enum {
	OBS_STATE_IDLE,	// starting point
	OBS_STATE_PAUSE,	
	OBS_STATE_RIGHT,	
	OBS_STATE_FORWARD,
	OBS_STATE_LEFT,
	OBS_STATE_TURNAROUND,
	OBS_STATE_CENTER,
} EObstacleState;

struct ObstacleResult {
  geometry_msgs::Twist cmd_vel;
	EObstacleState state;
	EObstacleState nextState;
	EObstacleResult result;
	std::string status;
	int count;
};

class ObstacleController {

  public:
    ObstacleController();
    void reset(void);
	ObstacleResult result;
	ObstacleResult execute(int obstacleDetected);
  private:
	int count;
ros::Time stateStartTime; // start time for states
	ros::Duration stateRunTime; // time since last state change
};

#endif /* Obstacle_CONTROLLER */
