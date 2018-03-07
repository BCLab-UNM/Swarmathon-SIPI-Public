#ifndef DROPOFCONTROLLER_H
#define DROPOFCONTROLLER_H

#include <apriltags_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include "GripperController.h"
#include "DrivingController.h"
namespace Dropoff {
	enum class ResultCode {
		SUCCESS = 0,
		BUSY,
		FAIL
	}; 

	enum class State {
		IDLE = 0,
		CENTER,
		FORWARDSTEER,
		FORWARD,
		DROP_CUBE,
		BACKUP
	}; 

	struct Result {
		geometry_msgs::Twist cmd_vel;
		CGripCmd grip;
		State state;
		State nextState;
		ResultCode result;
		std::string status;
	};

	class DropOffController
	{
		public:
			DropOffController(void);
			void reset(void);
			Result execute(const std::vector<geometry_msgs::Pose2D> &home_tags);
		private:
			Result result;
			ros::Time stateStartTime; // start time for states
			ros::Duration stateRunTime; // time since last state change
			bool targetLost;
			int missedTargetCount;
	};
} // end namespace
#endif // end header define
