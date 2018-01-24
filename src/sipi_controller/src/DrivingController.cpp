#include "DrivingController.h"
#include <angles/angles.h>

#define MIN_DIST_TO_GOAL 0.4
#define MAX_TURN_ONLY_ANGLE 0.6
#define MAX_ANGULAR_VEL 0.4
#define MAX_LINEAR_VEL 0.2

DrivingController::DrivingController() {
  status.busy = false;
  status.vel.linear = status.vel.yawError = 0.0;
	Kp_angular = 0.5;
}
double limit(double val, double limit) 
{
	if(val>limit) {
		return limit;
	} else if(val<-limit) {
		return  -limit;
	}
	return val;
}
/* this function produces cmd_Vel to drive to a specfied goal */
SDrivingStatus DrivingController::drive(
    geometry_msgs::Pose2D pose, // current pose of rover
    geometry_msgs::Pose2D goal, // goal pose
		float velocity  // meters/second
    )
{      
  // first calculate the distance and heading to the goal
  geometry_msgs::Pose2D diff;
  diff.x = goal.x - pose.x;
  diff.y = goal.y - pose.y;
  float distToGoal = sqrt(diff.x*diff.x + diff.y*diff.y);
  float headingToGoal = atan2(diff.y,diff.x);
  // errorYaw is the difference between the desired heading and the current one
  float errorYaw = angles::normalize_angle(headingToGoal - pose.theta);
  // if arrived at goal, then finished
  if (distToGoal < MIN_DIST_TO_GOAL) 
  {
    status.vel.linear = status.vel.yawError = 0.0;
    status.busy = false;
    // If heading to goal is too much to skid steer to then turn  
  } else if ( fabs(errorYaw) > MAX_TURN_ONLY_ANGLE) {
    // rotate but don't drive  0.05 is to prevent turning in reverse
    status.vel.linear = 0.0;
    status.vel.yawError = errorYaw * Kp_angular;
    status.busy = true;
  }
  // otherwise skid steer drive to goal
  // drive and turn simultaniously
  else {
    status.vel.linear = velocity;
		// skid steer has half the normal gain to prevent over correction
    status.vel.yawError = errorYaw * Kp_angular;
    status.busy = true;
  }
	status.headingToGoal = headingToGoal;
	status.errorYaw = errorYaw;
	status.distToGoal = distToGoal;
	status.vel.yawError = limit(status.vel.yawError, MAX_ANGULAR_VEL);
	status.vel.linear = limit(status.vel.linear, MAX_LINEAR_VEL);
  return status;
}

