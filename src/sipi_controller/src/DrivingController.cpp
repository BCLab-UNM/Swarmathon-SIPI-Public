#include "sipi_controller/DrivingController.h"
#include <angles/angles.h>

#define MIN_DIST_TO_GOAL 0.4
#define MAX_TURN_ONLY_ANGLE 0.6
#define MAX_ANGULAR_VEL 1.0
#define MAX_LINEAR_VEL 0.3

DrivingController::DrivingController() {
  status.busy = false;
  status.cmd_vel.linear.x = status.cmd_vel.angular.z = 0.0;
  Kp_angular = 0.5;
}

double limit(double val, double limit) {
  if(val>limit) {
    return limit;
  } else if(val<-limit) {
    return  -limit;
  }
  return val;
}

/* this function produces cmd_vel to drive to a specfied goal */
SDrivingStatus DrivingController::drive(
    geometry_msgs::Pose2D pose, // current pose of rover
    geometry_msgs::Pose2D goal, // goal pose
    float velocity  // meters/second
    )
{      
  // default zero velocity
  status.cmd_vel.linear.x = status.cmd_vel.angular.z = 0.0;
  // first calculate the distance and heading to the goal
  geometry_msgs::Pose2D diff;
  diff.x = goal.x - pose.x;
  diff.y = goal.y - pose.y;
  float distToGoal = sqrt(diff.x*diff.x + diff.y*diff.y);
  float headingToGoal = atan2(diff.y,diff.x);
  // errorYaw is the difference between the desired heading and the current one
  float errorYaw = angles::normalize_angle(headingToGoal - pose.theta);

  // if arrived at goal, then finished
  if (distToGoal < MIN_DIST_TO_GOAL) {
    status.busy = false;
    // If heading to goal is too much to skid steer to then turn  
  } else if ( fabs(errorYaw) > MAX_TURN_ONLY_ANGLE) {
    // rotate but don't drive
    status.cmd_vel.linear.x = 0.0;
    status.cmd_vel.angular.z = errorYaw * Kp_angular;
    status.busy = true;
  } else {
    // otherwise skid steer drive to goal
    // drive and turn simultaniously
    status.cmd_vel.linear.x = velocity;
    // skid steer has half the normal gain to prevent over correction
    status.cmd_vel.angular.z = errorYaw * Kp_angular;
    status.busy = true;
  }
  status.headingToGoal = headingToGoal;
  status.errorYaw = errorYaw;
  status.distToGoal = distToGoal;
  status.cmd_vel.angular.z = limit(status.cmd_vel.angular.z, MAX_ANGULAR_VEL);
  status.cmd_vel.linear.x = limit(status.cmd_vel.linear.x, MAX_LINEAR_VEL);
  return status;
}

