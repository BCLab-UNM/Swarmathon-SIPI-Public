#include "FindHomeController.h"
//  This is a constructor.  it gets called when the program first runs.
// this is where we need to create the list of goals. 
FindHomeController::FindHomeController() {
}
void FindHomeController::reset(void) 
{
	result.state = LOSER_STATE_IDLE;
	result.result = LOSER_RESULT_BUSY;
	result.status = "init";
	incrementingYawError = 0.3;
	startTime = ros::Time::now();
}

Finding_Home_Result FindHomeController::execute(
		int obstacleDetected, 
		bool homeVisible
		) 
{
	// time since last state change
	result.result = LOSER_RESULT_BUSY;
	dt = ros::Time::now() - startTime;
	switch(result.state) {
		case LOSER_STATE_IDLE:
			startTime = ros::Time::now();
			result.state = LOSER_STATE_SEARCH;
			result.vel.linear = 0.2;
			result.vel.yawError = 0.3;
			break;
		case LOSER_STATE_SEARCH:
			result.vel.linear = 0.2;
			result.vel.yawError = 0.3-(dt.toSec()/100.0);
			if(dt > ros::Duration(20.0)) {
				result.state = LOSER_STATE_IDLE;
				result.result = LOSER_RESULT_FAILED;
			}
			if(homeVisible) {
				result.state = LOSER_STATE_IDLE;
				result.result = LOSER_RESULT_SUCCESS;
			}
			break;
	}
	std::ostringstream ss;
	ss << " FindHome state: "<< result.state << std::setprecision(2) <<
		" time: " << dt <<
		" state: " << result.state <<
		" cmd: " << result.vel.linear << "," << result.vel.yawError <<
		" result= " << result.result;
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
