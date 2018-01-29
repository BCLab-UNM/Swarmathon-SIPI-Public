#ifndef GRIPPERCONTROLLER_H
#define GRIPPERCONTROLLER_H
#include <ros/ros.h>
typedef enum {
	WRIST_DOWN,
	WRIST_UP,
	WRIST_VERIFY
} EWristCmd;

class CGripCmd {
	public:
	CGripCmd(void) {wristPos=WRIST_UP; fingersOpen=false;};
	EWristCmd wristPos;
	bool fingersOpen;
};

class GripperController
{
	public:
		void move(CGripCmd cmd);
		GripperController(
				ros::NodeHandle _nh, 
				std::string publishedName
				); 
	private:
		ros::NodeHandle nh;
		ros::Publisher fingerAnglePublish;
		ros::Publisher wristAnglePublish;
};
#endif // end header define
