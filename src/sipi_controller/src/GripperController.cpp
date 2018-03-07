#include "sipi_controller/GripperController.h"
#include <std_msgs/Float32.h>
#define FINGER_OPEN_VALUE 0
#define FINGER_CLOSED_VALUE M_PI_2
#define WRIST_VERIFY_VALUE 0
#define WRIST_UP_VALUE 0.7
#define WRIST_DOWN_VALUE 1.1 

void GripperController::move(CGripCmd grip) 
{
	std_msgs::Float32 angle;
	if(grip.fingersOpen) {
		angle.data = FINGER_CLOSED_VALUE;
	} else {
		angle.data = FINGER_OPEN_VALUE;
	}
	fingerAnglePublish.publish(angle);

	if(grip.wristPos == WRIST_DOWN) {
		angle.data = WRIST_DOWN_VALUE;
	} else if(grip.wristPos == WRIST_UP) {
		angle.data = WRIST_UP_VALUE;
	} else if(grip.wristPos == WRIST_VERIFY) {
		angle.data = WRIST_VERIFY_VALUE;
	} else {
		ROS_WARN("Invalid command to wrist");
		angle.data = WRIST_UP_VALUE;
	}
	wristAnglePublish.publish(angle);
}
GripperController::GripperController(
		ros::NodeHandle _nh, 
		std::string publishedName
		) 
{
	nh = _nh;
	fingerAnglePublish = nh.advertise<std_msgs::Float32>((publishedName + "/fingerAngle/cmd"), 1, true);
	wristAnglePublish = nh.advertise<std_msgs::Float32>((publishedName + "/wristAngle/cmd"), 1, true);
}
