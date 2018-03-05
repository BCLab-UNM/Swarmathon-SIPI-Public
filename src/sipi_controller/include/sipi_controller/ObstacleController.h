#ifndef OBSTACLE_CONTROLLER_H
#define OBSTACLE_CONTROLLER_H
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

/**
 * This class implements the search control algorithm for the rovers. The code
 * here should be modified and enhanced to improve search performance.
 */
namespace Obstacle {

enum class ResultCode {
  SUCCESS = 0,
  BUSY,
  FAILED
};

enum class State : int {
  IDLE,	// starting point
  PAUSE,	
  RIGHT,	
  FORWARD,
  RIGHT2,
  LEFT,
  TURNAROUND,
  CENTER
};

struct Result {
  geometry_msgs::Twist cmd_vel;
  State state;
  State nextState;
  ResultCode result;
  std::string status;
  int count;
};

/*** external function to check ultrasound for any obstacle within a 
  certain distance */
bool obstacleDetected(const geometry_msgs::Point &ultrasound);

class ObstacleController {
  public:
    ObstacleController(void);
    void reset(void);
    Result result;
    Result execute(const geometry_msgs::Point &obstacles);
  private:
    int count;
    ros::Time stateStartTime; // start time for states
    ros::Duration stateRunTime; // time since last state change
};

}
#endif /* Obstacle_CONTROLLER */
