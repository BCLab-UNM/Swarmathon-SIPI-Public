#include "SearchController.h"

// This is a constructor. it gets called when the program first runs.
// this is where we need to create the list of goals.

SearchController::SearchController() 
{
	currentGoal = 0;
	geometry_msgs::Pose2D goal;
	int robotNumber = 1;
	float x; 
	float y;
	 	currentGoal = 0;
	 	if(robotNumber == 1) {
			x = -7.1; 
			y = 7.1;
	 	} else  if(robotNumber == 2) {
			x = 7.1;
			y = -7.1;
		} else  if(robotNumber == 3) {
			x = -7.1; 
			y = -7.1;
		}

	// step 2
	goal.x = x; goal.y = y;
	goals.push_back(goal);

	//step3
	x = x +0.5;
	for ( ; x<7.1 ; x=x+0.5 ) {
		//step4
		goal.x=x; goal.y = y;
		goals.push_back(goal);

		//step5
		y=-y;

		//step6
		goal.x = x; goal.y = y;
		goals.push_back(goal);
	}
	
	//step7
	for( ; x<7.0; x=x+0.5){
	}

	for (size_t i=0; i<goals.size();++i){
		std::cout<<goals[i]<<std::endl;
	}
}
/**
 * This code implements a basic random walk search.
 */
geometry_msgs::Pose2D SearchController::search(geometry_msgs::Pose2D currentLocation) {
  currentGoal++;
  if (currentGoal > goals.size()-1) {
     currentGoal = 0;
  }
  return goals[currentGoal];	
}

/**
 * Continues search pattern after interruption. For example, avoiding the
 * center or collisions.
 */
geometry_msgs::Pose2D SearchController::continueInterruptedSearch(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D oldGoalLocation) {
  geometry_msgs::Pose2D newGoalLocation;

  //remainingGoalDist avoids magic numbers by calculating the dist
  double remainingGoalDist = hypot(oldGoalLocation.x - currentLocation.x, oldGoalLocation.y - currentLocation.y);

  //this of course assumes random walk continuation. Change for diffrent search methods.
  newGoalLocation.theta = oldGoalLocation.theta;
  newGoalLocation.x = currentLocation.x + (0.50 * cos(oldGoalLocation.theta)); //(remainingGoalDist * cos(oldGoalLocation.theta));
  newGoalLocation.y = currentLocation.y + (0.50 * sin(oldGoalLocation.theta)); //(remainingGoalDist * sin(oldGoalLocation.theta));

  return newGoalLocation;
}
