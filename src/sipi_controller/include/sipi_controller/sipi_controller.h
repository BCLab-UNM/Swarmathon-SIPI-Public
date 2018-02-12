#ifndef _SIPI_CONTROLLER_INCLUDE_SIPI_CONTROLLER_SIPI_CONTROLLER_H
#define _SIPI_CONTROLLER_INCLUDE_SIPI_CONTROLLER_SIPI_CONTROLLER_H

#include <ros/ros.h>

// ROS libraries
#include <angles/angles.h>
#include <random_numbers/random_numbers.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

// ROS messages
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <apriltags_ros/AprilTagDetectionArray.h>

// Include Controllers
#include "sipi_controller/DrivingController.h"
#include "sipi_controller/PickUpController.h"
#include "sipi_controller/DropOffController.h"
#include "sipi_controller/SearchController.h"
#include "sipi_controller/ObstacleController.h"
#include "sipi_controller/GripperController.h"
#include "sipi_controller/FindHomeController.h"
#include "sipi_controller/AvoidHome.h"
#include "sipi_controller/Localization.h"
#include "sipi_controller/targetFunctions.h"

#define PICKUP_TIMEOUT 20
using namespace std;

class sipi_controller {
  public: 
    sipi_controller(const std::string &name, int argc, char** argv);

    // Mobility Logic Functions
    void sendDriveCommand(CDriveCmd vel);
    void sendDriveCommand(float linear, float yawError);
    void publishState(void);
    int countRovers(void);
    void publishHeartBeatTimerEventHandler(const ros::TimerEvent&);

    // state machine states
    typedef enum {
      STATE_MACHINE_MANUAL,     // in manual mode
      STATE_MACHINE_INIT,				// initialization 
      STATE_MACHINE_SEARCH,  // driving to points in search pattern
      STATE_MACHINE_RETURN,  // returning to base after pickup
      STATE_MACHINE_FIND_HOME,  // trying to find home
      STATE_MACHINE_OBSTACLE,  // avoiding an obstacle
      STATE_MACHINE_AVOID_HOME,  // avoiding going through home 
      STATE_MACHINE_PICKUP,     // picking up cube
      STATE_MACHINE_DROPOFF     // dropping off cube at ome base
    } EStateMachineStates;

    // Publishers
    ros::Publisher stateMachinePublish;
    ros::Publisher status_publisher;
    ros::Publisher infoLogPublisher;
    ros::Publisher driveControlPublish;
    ros::Publisher heartbeatPublisher;

    // Subscribers
    ros::Subscriber joySubscriber;
    ros::Subscriber modeSubscriber;
    ros::Subscriber obstacleSubscriber;
    ros::Subscriber targetSubscriber;

    // Timers
    ros::Timer stateMachineTimer;
    ros::Timer publish_status_timer;
    ros::Timer targetDetectedTimer;
    ros::Timer publish_heartbeat_timer;

    // OS Signal Handler
    void sigintEventHandler(int signal);

    //Callback handlers
    void joyCmdHandler(const sensor_msgs::Joy::ConstPtr& message);
    void modeHandler(const std_msgs::UInt8::ConstPtr& message);
    void targetHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& 
        tagInfo);
    void obstacleHandler(const std_msgs::UInt8::ConstPtr& message);
    void stateMachine(const ros::TimerEvent&);
    void publishStatusTimerEventHandler(const ros::TimerEvent& event);
    void targetDetectedReset(const ros::TimerEvent& event);

    // GLOBAL VARIABLES
    // these should really be in a class
    int obstacleDetected = 0;
    int obstacleCount = 0;
    geometry_msgs::Twist velocity;
    char host[128];
    std::string name;
    char prev_state_machine[128];
    apriltags_ros::AprilTagDetectionArray tagDetectionArray;
    SDrivingStatus drivingResult;
    PickUpResult pickupResult;
    DropOffResult dropoffResult;
    Finding_Home_Result findResult;
    ObstacleResult obstacleResult;
    AvoidResult avoidResult;
    CDriveCmd vel;
    ros::Time stateStartTime;
    ros::Duration stateRunTime;
    EStateMachineStates state = STATE_MACHINE_MANUAL;
    EStateMachineStates prevState = STATE_MACHINE_MANUAL;
    EStateMachineStates avoidPrevState = STATE_MACHINE_MANUAL;
    EStateMachineStates nextState = STATE_MACHINE_MANUAL;
    // controllers
    DrivingController drivingController;
    PickUpController pickUpController;
    DropOffController dropoffController;
    SearchController searchController;
    ObstacleController obstacleController;
    AvoidController avoidController;
    FindHomeController findHomeController;
    GripperController *gripperController;
    Localization *localization;
    int numberOfRovers = 1; // how many rovers are active
    int currentMode = 0;
    bool carryingCube = false;
    float stateMachinePeriod = 0.1; // time between the state machine loop
    float status_publish_interval = 1;
    // watchdog  timeout means something has died and we need to start over
    float watchdogTimeout = 60;
    bool targetDetected = false;
    bool targetCollected = false;
    bool homeVisible = false;
    bool blockVisible = false;
    int missedHomeCount = 0;
    bool homeSeen = false; // allows it to be missed a few times before going false
    // is state machine processing?  if so hold off on asyn updates from topics
    bool machineBusy = true;
    // Set true when we are insie the center circle and we need to drop the block,
    // back out, and reset the boolean cascade.
    bool reachedCollectionPoint = false;
    // used for calling code once but not in main
    bool init = false;
    bool avoidingObstacle = false;
    std_msgs::String msg;
    // Numeric Variables for rover positioning
    geometry_msgs::Pose2D currentPoseOdom;
    geometry_msgs::Pose2D currentPoseArena;
    geometry_msgs::Pose2D goalPoseOdom;
    geometry_msgs::Pose2D goalPoseArena;
    void setGoalPose(geometry_msgs::Pose2D pose);
    void setGoalPose(double x, double y);
    sensor_msgs::Joy lastJoyCmd;
};

#endif // _SIPI_CONTROLLER_INCLUDE_SIPI_CONTROLLER_SIPI_CONTROLLER_H
