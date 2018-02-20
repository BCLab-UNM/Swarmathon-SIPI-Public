#ifndef PICKUPCONTROLLER_H
#define PICKUPCONTROLLER_H
#define HEADERFILE_H
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/Twist.h>
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
    CENTER_COARSE,	// turn so that cube is in center
    DIST,	//move to the correct distance
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
          const apriltags_ros::AprilTagDetectionArray& targets
          ); 

      void reset();
    private:
      //yaw error to target block 
      double blockYawError;
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
  };

}
#endif // end header define
