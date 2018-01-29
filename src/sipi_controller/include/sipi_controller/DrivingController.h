#ifndef DRIVINGCONTROLLER_H
#define DRIVINGCONTROLLER_H
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
class CDriveCmd {
	public:
	CDriveCmd(void) {linear=yawError=0.0;};
  float linear;
  float yawError;
};

typedef struct {
  bool  busy;
	float errorYaw;
	CDriveCmd vel;
	float headingToGoal;
	float distToGoal;
} SDrivingStatus;
// limit a value to +/- limit
double limit(double val, double limit);

class DrivingController

{
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
