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
    State nextState;
    ResultCode result;
    std::string status;
    int count;
  };

  class Controller {

    public:
      Controller();
      void reset(void);
      Result result;
      Result execute(bool homeVisible);
    private:
      int count;
      ros::Time stateStartTime; // start time for states
      ros::Duration stateRunTime; // time since last state change
  };
} // end namespace
#endif 
