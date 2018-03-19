#ifndef PICKUPCONTROLLER_H
#define PICKUPCONTROLLER_H
#define HEADERFILE_H
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <ros/ros.h>
#include "GripperController.h"
#include "DrivingController.h"
#include "targetFunctions.h"
namespace PickupController {

  enum class ResultCode : int {
    SUCCESS = 0,
    BUSY,
    FAILED
  };

  enum class State : int {
    IDLE,	// starting point
    CENTER,	// turn so that cube is in center
    DISTANCE,	//move to the correct distance
    FORWARD,	// go forward to scoop up cube
    PICKUP,		// pick up cube
    VERIFY,		// check if successful
    BACKUP		// back up a little to reacquire lost target
  };

  struct Result {
    geometry_msgs::Twist cmd_vel;
    CGripCmd grip;
    bool pickedUp;
    bool giveUp;
    State state;
    State nextState;
    ResultCode result;
    std::string status;
  };

  class PickUpController
  {
    public:
      PickUpController();
      ~PickUpController();

      Result execute(
          int obstacleDetection,
          const std::vector<geometry_msgs::Pose2D> targets
          ); 

      void reset();
      bool ignore_cubes(void) {return ignore_cubes_;};
    private:
      //yaw angle to target block 
      double blockYaw;
      //distance to target block from front of robot
      double blockDist;
      double distErr;

      //struct for returning data to mobility
      Result result;

      ros::Time stateStartTime; // start time for states
      ros::Duration stateRunTime; // time since last state change
      geometry_msgs::Pose tagPose;
      int missedTargetCount;
      // flag to indicate that the target is no longer visible
      bool targetLost;
      int fail_count;
      bool ignore_cubes_;
  };

}
#endif // end header define
