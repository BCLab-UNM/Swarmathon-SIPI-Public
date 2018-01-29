#ifndef SEARCH_CONTROLLER
#define SEARCH_CONTROLLER
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <random_numbers/random_numbers.h>

/**
 * This class implements the search control algorithm for the rovers. The code
 * here should be modified and enhanced to improve search performance.
 */
class SearchController {

	public:

		SearchController();
		// start the pattern over
		void reset(void) {currentGoal = 0;};
		// create the pattern for the first time
		void createPattern(
				geometry_msgs::Pose2D startingPose,
				int numSwarmies);
		// get the next search goal
		geometry_msgs::Pose2D getNextGoal(void);
		// get the current goal
		geometry_msgs::Pose2D getCurrentGoal(void);

	private:
		void fillPatternSpokes(float startAngle, float endAngle, float dist);
		void fillPatternSpokesSquare(float startAngle, float endAngle);
		std::vector<geometry_msgs::Pose2D> goals;
		int currentGoal;

};

#endif /* SEARCH_CONTROLLER */
