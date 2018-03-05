#include "sipi_controller/FindHomeController.h"
//  This is a constructor.  it gets called when the program first runs.
// this is where we need to create the list of goals. 
using namespace FindHome;

Controller::Controller() {
}
void Controller::reset(void) 
{
	result.state = State::IDLE;
	result.result = ResultCode::BUSY;
	result.status = "init";
	startTime = ros::Time::now();
}

Result Controller::execute(
		int obstacleDetected, 
		bool homeVisible
		) 
{
	// time since last state change
	result.result = ResultCode::BUSY;
	dt = ros::Time::now() - startTime;
	switch(result.state) {
		case State::IDLE:
			startTime = ros::Time::now();
			result.state = State::SEARCH;
			result.cmd_vel.linear.x = 0.0;
			result.cmd_vel.angular.z = 0.0;
			break;
		case State::SEARCH:
			result.cmd_vel.linear.x = 0.2;
			result.cmd_vel.angular.z = 0.5-(dt.toSec()/100.0);
			if(dt > ros::Duration(20.0)) {
				result.state = State::IDLE;
				result.result = ResultCode::FAILED;
			}
			if(homeVisible) {
				result.state = State::IDLE;
				result.result = ResultCode::SUCCESS;
			}
			break;
	}
	std::ostringstream ss;
	ss << " FindHome state: "<< (int)result.state << std::setprecision(2) <<
		" time: " << dt <<
		" cmd: " << result.cmd_vel.linear.x << "," << result.cmd_vel.angular.z <<
		" result= " << (int)result.result;
	result.status = ss.str();
	return result;
}

#if 0
	geometry_msgs::Pose2D goal;
	// choose random heading
	
	for (int i = 0; i < 10; i++)
	{
	float the = 1;
	double angle = pow(the,i);
//	double angle =(float)(rand() % 1000)/1000.0*(2.0*M_PI);

	//select new position from current location
	goal.x = currentLocation.x + (1.0 * cos(angle));
	goal.y = currentLocation.y + (1.0 * sin(angle));
	
	return goal;
	}
}
#endif
