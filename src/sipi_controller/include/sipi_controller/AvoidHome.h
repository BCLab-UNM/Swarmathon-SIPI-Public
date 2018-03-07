#ifndef AVOID_HOME_H
#define AVOID_HOME_H
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <random_numbers/random_numbers.h>
#include "DrivingController.h"
namespace AvoidHome {
  enum class ResultCode {
    SUCCESS = 0,
    BUSY,
    FAILED
  };

  enum class State {
    IDLE,	// starting point
    PAUSE,	
    RIGHT,	
    FORWARD
  };

  struct Result {
    geometry_msgs::Twist cmd_vel;
    State state;
    State next_state_;
    ResultCode result;
    std::string status;
    int count;
  };

  class Controller {

    public:
      Controller();
      void reset(void);
      Result result;
      Result execute(const std::vector<geometry_msgs::Pose2D> &home_tags);
    private:
      int count;
      ros::Time stateStartTime; // start time for states
      ros::Duration stateRunTime; // time since last state change
  };
} // end namespace
#endif 
