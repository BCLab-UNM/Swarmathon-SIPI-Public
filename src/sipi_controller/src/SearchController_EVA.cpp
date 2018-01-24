#include "SearchController.h"
#include <geometry_msgs/Pose2D.h>
SearchController::SearchController() {
  currentGoal = 0;
  geometry_msgs::Pose2D goal;
goal.x = 0; goal.y = 0;
goals.push_back(goal);
  goal.x = 1; goal.y = 1;
   goals.push_back(goal);
  goal.x = 2; goal.y = 0;
   goals.push_back(goal);
  goal.x = 0; goal.y = -2;
   goals.push_back(goal);
  goal.x = -3; goal.y = 1;
   goals.push_back(goal);
  goal.x = 2; goal.y = 6;
   goals.push_back(goal);
  goal.x = 10; goal.y = -2;
   goals.push_back(goal);

 
	for(size_t i=0;i<goals.size();++i) {
		std::cout << goals[i];
	}
}

/**
 * This code implements a basic random walk search.
 */
geometry_msgs::Pose2D SearchController::search(geometry_msgs::Pose2D currentLocation) 
{ 
	geometry_msgs::Pose2D g = goals[currentGoal];
	currentGoal++;
	
	if(currentGoal > goals.size()-1){
	  currentGoal = 0;
	}
	std::cout << "measured "<< g << " goal "<< goals[currentGoal];
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
