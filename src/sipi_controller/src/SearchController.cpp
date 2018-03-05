#include "sipi_controller/SearchController.h"

#define MAX_DIST 5  // how far from center to go
#define MAX_DIST_FINALS 8  // how far from center to go
#define MIN_DIST 1  // how close to center to return
#define ANGLE_STEP 0.1 // radians between spokes
// if 0 draw circle if 1 draw square
#define USE_SQUARE 0

// spokes with constant radius, misses corners but avoids walls
void SearchController::fillPatternSpokes(float startAngle, float endAngle, float dist) 
{
	geometry_msgs::Pose2D goal;
	for(float a = startAngle; a<=endAngle ; a+= ANGLE_STEP) {
#if 0
		goal.x = dist*cos(a);
		goal.y = dist*sin(a);
		goals.push_back(goal);
#endif

	
	if (a > 0.5 && a < 0.81)
	{
		dist = dist + 0.33;
	}
	else if (a > 0.85 && a < 1.0)
	{
		dist = dist - 0.33;
	}
	else dist = dist;

		goal.x = dist*cos(a);
		goal.y = dist*sin(a);
		goals.push_back(goal);

	}
}
geometry_msgs::Pose2D calcEndpoint(float a)
{
	geometry_msgs::Pose2D goal;
	if(a <= M_PI_4) {
		goal.x = MAX_DIST;
		goal.y = MAX_DIST*tan(a);
	} else if(M_PI_4 <= a && a <= 3.0*M_PI_4) {
		goal.x = -MAX_DIST*tan(a - M_PI_2);
		goal.y = MAX_DIST;
	} else if(3.0*M_PI_4 <= a && a <= 5.0*M_PI_4) {
		goal.x = -MAX_DIST;
		goal.y = -MAX_DIST*tan(a - M_PI);
	} else if(5.0*M_PI_4 <= a && a <= 7.0*M_PI_4) {
		goal.x = MAX_DIST*tan(a - 3.0*M_PI_2);
		goal.y = -MAX_DIST;
	} else {
		goal.x = MAX_DIST;
		goal.y = MAX_DIST*tan(a-2.0*M_PI);
	}
	return goal;
}
// same as spokes but goes to corners
// calc end point intersection of square and circle at angle a
void SearchController::fillPatternSpokesSquare(
		float startAngle, 
		float endAngle) 
{
	geometry_msgs::Pose2D home;
	home.x = 0; home.y = 0; goals.push_back(home);
	for(float a = startAngle; a<=endAngle ; a+= ANGLE_STEP) {
		goals.push_back(calcEndpoint(a));
	}
}
SearchController::SearchController() {
	currentGoal = 0;
}
/*
// print to screen (just during testing)
for(size_t i=0;i<goals.size();++i) {
std::cout << goals[i];
}
 */

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
	ROS_INFO_STREAM("New Goal = "<<goal);
	return goal;
}

