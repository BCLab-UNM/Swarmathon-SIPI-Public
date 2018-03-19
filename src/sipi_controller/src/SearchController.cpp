#include "sipi_controller/SearchController.h"

#define MAX_DIST 10  // how far from center to go
#define MAX_DIST_FINALS 20  // how far from center to go
#define MIN_DIST 1  // how close to center to return
#define ANGLE_STEP 0.1 // radians between spokes
// if 0 draw circle if 1 draw square
#define USE_SQUARE 0
SearchController::SearchController() {
       currentGoal = 0;
}

// spokes with constant radius, misses corners but avoids walls
void SearchController::fillPatternSpokes(float startAngle, float endAngle, float dist) 
{
	geometry_msgs::Pose2D goal;
	for(float a = startAngle; a <= M_PI ; a+= ANGLE_STEP) {
		goal.x = dist*cos(a);
		goal.y = dist*sin(a);
		goals.push_back(goal);
	}
}

void SearchController::createPattern(
		geometry_msgs::Pose2D startingPose,
		int numSwarmies)
{
	// starting angle is just the angle from 0 to where the 
	// swarmie started
	float startAngle = atan2(startingPose.y,startingPose.x);
	ROS_INFO_STREAM("Num Rovers = "<<numSwarmies<<
			" my starting angle = " << startAngle/M_PI*180.0);
	// end angle divides the arena by number of swarmies
	// might be better to just go full circle and let them progress
	// around in order to avoid missing zones between assignments?
	float endAngle = startAngle + M_PI*2.0;
	if(numSwarmies == 6) {
		fillPatternSpokes(startAngle, endAngle, MAX_DIST_FINALS);
	} else {
		fillPatternSpokes(startAngle, endAngle, MAX_DIST);
	}
}

geometry_msgs::Pose2D SearchController::getCurrentGoal(void) {
	return  goals[currentGoal];
}

geometry_msgs::Pose2D SearchController::getNextGoal(void) {
	currentGoal++;
	if(currentGoal > goals.size()-1) {
		currentGoal = 0;
	}
	geometry_msgs::Pose2D goal = goals[currentGoal];
	ROS_INFO_STREAM("Search controller New Goal = "<<goal);
	return goal;
}

