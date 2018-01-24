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

    // performs search pattern
    geometry_msgs::Pose2D search(geometry_msgs::Pose2D currentLocation);

		void fillPattern(float x_start, float y_start, float x_end, float y_end);
		void fillPatternSpokes(float startAngle, float endAngle);
		void fillPatternSpokes2(float startAngle, float endAngle);
    // continues search pattern after interruption
    geometry_msgs::Pose2D continueInterruptedSearch(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D oldGoalLocation);
    void reset(void) {currentGoal = 0;};
  private:
		std::vector<geometry_msgs::Pose2D> goals;
		int currentGoal;

    random_numbers::RandomNumberGenerator* rng;
};

#endif /* SEARCH_CONTROLLER */
