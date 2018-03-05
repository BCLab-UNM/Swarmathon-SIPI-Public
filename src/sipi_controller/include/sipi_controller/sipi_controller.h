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
#include <geometry_msgs/Point.h>
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

using namespace std;

class sipi_controller {
  public: 
    sipi_controller(const std::string &name, int argc, char** argv);

    void sendDriveCommand(geometry_msgs::Twist vel);
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
    ros::Subscriber sonarLeftSubscriber;
    ros::Subscriber sonarCenterSubscriber;
    ros::Subscriber sonarRightSubscriber;
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
    void sonarLeftHandler(const sensor_msgs::Range::ConstPtr& message);
    void sonarCenterHandler(const sensor_msgs::Range::ConstPtr& message);
    void sonarRightHandler(const sensor_msgs::Range::ConstPtr& message);
    void stateMachine(const ros::TimerEvent&);
    void publishStatusTimerEventHandler(const ros::TimerEvent& event);
    void targetDetectedReset(const ros::TimerEvent& event);

    bool obstacle_detected;
    int obstacle_count;
    geometry_msgs::Twist cmd_vel_;
    char host[128];
    std::string name;
    char prev_state_machine[128];
    apriltags_ros::AprilTagDetectionArray tagDetectionArray;
    SDrivingStatus drivingResult;
    PickupController::Result pickupResult;
    DropOffResult dropoffResult;
    FindHome::Result findResult;
    Obstacle::Result obstacleResult;
    AvoidResult avoidResult;
    ros::Time stateStartTime;
    ros::Duration stateRunTime;
    EStateMachineStates state;
    EStateMachineStates prevState;
    EStateMachineStates avoidPrevState;
    EStateMachineStates nextState;
    // controllers
    DrivingController drivingController;
    PickupController::PickUpController pickUpController;
    DropOffController dropoffController;
    SearchController searchController;
    Obstacle::ObstacleController obstacleController;
    AvoidController avoidController;
    FindHome::Controller findHomeController;
    GripperController *gripperController;
    Localization *localization;
    int numberOfRovers; // how many rovers are active
    int currentMode;
    bool carryingCube;
    float stateMachinePeriod; // time between the state machine loop
    float status_publish_interval;
    // watchdog  timeout means something has died and we need to start over
    float watchdogTimeout;
    bool targetDetected;
    bool targetCollected;
    bool homeVisible;
    bool blockVisible;
    int missedHomeCount;
    bool homeSeen; // allows it to be missed a few times before going false
    // is state machine processing?  if so hold off on asyn updates from topics
    bool machineBusy;
    // Set true when we are insie the center circle and we need to drop the block,
    // back out, and reset the boolean cascade.
    bool reachedCollectionPoint;
    // used for calling code once but not in main
    bool init;
    bool avoidingObstacle;
    std_msgs::String msg;
    // Numeric Variables for rover positioning
    geometry_msgs::Pose2D currentPoseOdom;
    geometry_msgs::Pose2D currentPoseArena;
    geometry_msgs::Pose2D goalPoseOdom;
    geometry_msgs::Pose2D goalPoseArena;
    void setGoalPose(geometry_msgs::Pose2D pose);
    void setGoalPose(double x, double y);
    sensor_msgs::Joy lastJoyCmd;
    geometry_msgs::Point ultrasound;
};

#endif // _SIPI_CONTROLLER_INCLUDE_SIPI_CONTROLLER_SIPI_CONTROLLER_H
