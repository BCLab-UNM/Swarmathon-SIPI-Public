#ifndef DRIVINGCONTROLLER_H
#define DRIVINGCONTROLLER_H
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>

typedef struct {
  // flag to indicate still busy with previous command
  bool  busy;
  // resulting velocity command
  geometry_msgs::Twist cmd_vel;
  // angle to goal from current pose
  float headingToGoal;
  // angle difference from heading to goal and current heading
  float errorYaw;
  // distance from current pose to goal
  float distToGoal;
} SDrivingStatus;

// limit a value to +/- limit
double limit(double val, double limit);

class DrivingController {
  public:
    DrivingController();
    SDrivingStatus status;
    SDrivingStatus drive(
        geometry_msgs::Pose2D pose, // current pose of rover
        geometry_msgs::Pose2D goal, // goal pose
        float velocity = 0.5  // meters/second
        );
  private:
    float Kp_angular; // proportional gain for in-place turing
};
#endif // end header define
