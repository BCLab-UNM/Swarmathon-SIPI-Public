#include "SearchController.h"

//  This is a constructor.  it gets called when the program first runs.
// this is where we need to create the list of goals. 
#define MAX_DIST 5
#define X_STEP 0.5
#define ANGLE_STEP 0.2
void SearchController::fillPattern(float x_start, float y_start, float x_end, float y_end) 
{
	geometry_msgs::Pose2D goal;
	goal.x = 0; goal.y = 0; goals.push_back(goal);
	bool goingUp = true;
	for(float x=x_start ; x<x_end ; x+=X_STEP) {
		if(goingUp) {
			goal.x = x; goal.y = y_start; goals.push_back(goal);
			goal.x = x; goal.y = y_end; goals.push_back(goal);
		} else {
			goal.x = x; goal.y = y_end; goals.push_back(goal);
			goal.x = x; goal.y = y_start; goals.push_back(goal);
		}
		goingUp = !goingUp;
	}
}
// spokes with constant radius, misses corners
void SearchController::fillPatternSpokes(float startAngle, float endAngle) 
{
	geometry_msgs::Pose2D goal;
	goal.x = 0; goal.y = 0; goals.push_back(goal);
	for(float a = startAngle; a<=endAngle ; a+= ANGLE_STEP) {
		goal.x = MAX_DIST*cos(a);
		goal.y = MAX_DIST*sin(a);
		goals.push_back(goal);
		goal.x = 0; goal.y = 0; goals.push_back(goal);
	}
}
// same as spokes but goes to corners
// calc end point intersection of square and circle at angle a
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
void SearchController::fillPatternSpokes2(float startAngle, float endAngle) 
{
	geometry_msgs::Pose2D home;
	home.x = 0; home.y = 0; goals.push_back(home);
	for(float a = startAngle; a<=endAngle ; a+= ANGLE_STEP) {
		goals.push_back(calcEndpoint(a));
		goals.push_back(calcEndpoint(a+ANGLE_STEP/2.0));
		goals.push_back(home);
	}
}
SearchController::SearchController() {
	currentGoal = 0;
	// step 1
	int robotNumber = 1;
	bool finals = false;
	if(!finals) {
		if(robotNumber == 1) {
	//		fillPattern(0.5,0.5,MAX_DIST,MAX_DIST);
			fillPatternSpokes2(0, M_PI);
		} else 	if(robotNumber == 2) {
			fillPattern(0.5,0.5,-MAX_DIST,MAX_DIST);
		} else 	if(robotNumber == 3) {
			fillPattern(-MAX_DIST,0.5,MAX_DIST,MAX_DIST);
		}
	}
	for(size_t i=0;i<goals.size();++i) {
		std::cout << goals[i];
	}
}

/**
 * This code implements a basic lookup search.
 */
geometry_msgs::Pose2D SearchController::search(geometry_msgs::Pose2D currentLocation) {
	currentGoal++;
	if(currentGoal > goals.size()-1) {
		currentGoal = 0;
	}
	geometry_msgs::Pose2D goal = goals[currentGoal];
	ROS_INFO_STREAM("New Goal = "<<goal);
	return goal;
}

/**
 * Continues search pattern after interruption. For example, avoiding the
 * center or collisions.
 */
geometry_msgs::Pose2D SearchController::continueInterruptedSearch(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D oldGoalLocation) {
	return goals[currentGoal];
}
