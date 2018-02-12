#include "sipi_controller/sipi_controller.h"

///////////////////////////////////////////////////////////////////
sipi_controller::sipi_controller(
    const std::string &name, 
    int argc, char** argv) {

  ros::NodeHandle mNH;

  gripperController = new GripperController(mNH, name);
  localization = new Localization(name, mNH);

  joySubscriber = mNH.subscribe((name + "/joystick"), 10, 
      &sipi_controller::joyCmdHandler, this);
  modeSubscriber = mNH.subscribe((name + "/mode"), 1, 
      &sipi_controller::modeHandler, this);
  targetSubscriber = mNH.subscribe((name + "/targets"), 10, 
      &sipi_controller::targetHandler, this);
  obstacleSubscriber = mNH.subscribe((name + "/obstacle"), 10, 
      &sipi_controller::obstacleHandler, this);

  status_publisher = mNH.advertise<std_msgs::String>(
      (name + "/status"), 10, true);
  stateMachinePublish = mNH.advertise<std_msgs::String>(
      (name + "/state_machine"), 10, true);
  infoLogPublisher = mNH.advertise<std_msgs::String>("/infoLog", 10, true);
  driveControlPublish = mNH.advertise<geometry_msgs::Twist>(
      (name + "/driveControl"), 10);


  publish_heartbeat_timer = mNH.createTimer(ros::Duration(2.0), 
      &sipi_controller::publishHeartBeatTimerEventHandler, this);
  heartbeatPublisher = mNH.advertise<std_msgs::String>(
      (name + "/behaviour/heartbeat"), 1, true);


  publish_status_timer = mNH.createTimer(
      ros::Duration(status_publish_interval), 
      &sipi_controller::publishStatusTimerEventHandler, this);
  stateMachineTimer = mNH.createTimer(
      ros::Duration(stateMachinePeriod), 
      &sipi_controller::stateMachine, this);
  targetDetectedTimer = mNH.createTimer(ros::Duration(1), 
      &sipi_controller::targetDetectedReset, this);

  std_msgs::String msg;
  msg.data = "Log Started";
  infoLogPublisher.publish(msg);

  stateStartTime =  ros::Time::now();
}

void sipi_controller::stateMachine(const ros::TimerEvent&) {
  // first find where we are
  // to ignore GPS and visual, set both to Odom
  currentPoseOdom = localization->getPoseOdom();
  currentPoseArena = localization->getPoseArena();
  // this allows the target to dissappear briefly but not consider it lost
  // at first (reset) homeSeen is false. but then it only becomes false
  // again if the target is missed multiple times
  homeVisible = checkForTarget(tagDetectionArray, HOME_TAG_ID);
  blockVisible = checkForTarget(tagDetectionArray, BLOCK_TAG_ID);
  if(homeVisible) {
    missedHomeCount = 0;
    homeSeen = true;
  } else {
    missedHomeCount++;
    if(missedHomeCount > 10) {
      homeSeen = false;
    }
  }
  if(obstacleDetected > 0 && obstacleDetected < 4) {
    obstacleCount++;
  } else {
    obstacleCount = 0;
  }
  /* 
     if we were doing something and try to drive over home
   */
#if 1
  if((homeSeen) && (state == STATE_MACHINE_SEARCH)) {
    // remember what we were doing
    if(drivingResult.vel.linear > 0) {
      avoidPrevState = state;
      avoidController.reset();
      nextState = STATE_MACHINE_AVOID_HOME;
    }
  }
#endif
  /* 
     if we were doing something and hit an obstacle, then 
     switch to obstacle state
   */
#if 1
  if((obstacleCount > 5) && (state != STATE_MACHINE_OBSTACLE) &&
      (state != STATE_MACHINE_MANUAL) && (state != STATE_MACHINE_PICKUP)
      && (state != STATE_MACHINE_DROPOFF)) {
    // remember what we were doing
    prevState = state;
    obstacleController.reset();
    nextState = STATE_MACHINE_OBSTACLE;
  }
#endif
  // time since last state change
  if(nextState != state) {
    stateStartTime =  ros::Time::now();
    state = nextState;
  }
  stateRunTime = ros::Time::now() - stateStartTime;
  machineBusy = true;
  // default commands for move and gripper
  vel.linear = vel.yawError = 0.0;
  CGripCmd grip;

  /***
    This is the watchdog timer.  if any state hangs for more than 
    the watchdog time, then everything starts over
   */
  if(stateRunTime > ros::Duration(watchdogTimeout) && 
      state != STATE_MACHINE_MANUAL) {
    ROS_ERROR_STREAM("Statemachine watchdog triggered!");
    setGoalPose(0,0);
    nextState = STATE_MACHINE_RETURN;
  }

  if (currentMode == 0 || currentMode == 1) {
    nextState = STATE_MACHINE_MANUAL;
  }
  // main state machine
  switch(state) {
    case STATE_MACHINE_MANUAL:
      if (currentMode == 2 || currentMode == 3) {
        nextState = STATE_MACHINE_INIT;
      }
      break;
    case STATE_MACHINE_INIT:
      if (stateRunTime > ros::Duration(1)) {
        init = true;
        numberOfRovers = countRovers();
        //currentPoseArena = localization->getPoseArena();
        searchController.createPattern(currentPoseArena, numberOfRovers);
        searchController.reset();
        setGoalPose(searchController.getNextGoal());
        nextState = STATE_MACHINE_SEARCH;
      }
      break;
    case STATE_MACHINE_SEARCH: 
      // go through search pattern goals
      if(stateRunTime > ros::Duration(30)) {
        ROS_WARN("Search timeout, going to next pose");
        setGoalPose(searchController.getNextGoal());
      }
      drivingResult = drivingController.drive(currentPoseOdom, 
          goalPoseOdom, 0.2);
      vel = drivingResult.vel;
      if(blockVisible && !homeSeen) {
        pickUpController.reset();  
        nextState = STATE_MACHINE_PICKUP;
      } else if(!drivingResult.busy) {   // arrived at goal
        setGoalPose(0,0);
        nextState = STATE_MACHINE_RETURN;
      }
      break;
    case STATE_MACHINE_AVOID_HOME: 
      avoidResult = avoidController.execute(homeSeen);
      vel = avoidResult.vel;
      if(avoidResult.result != AVOID_RESULT_BUSY) {
        nextState = avoidPrevState;
      }
      break;
      // avoid an obstacle then continue
    case STATE_MACHINE_OBSTACLE: 
      obstacleResult = obstacleController.execute(obstacleDetected);
      vel = obstacleResult.vel;
      if(obstacleResult.result == OBS_RESULT_SUCCESS) {
        nextState = prevState;
      } else if(obstacleResult.result == OBS_RESULT_FAILED) {
        findHomeController.reset();
        nextState = STATE_MACHINE_FIND_HOME;
      }
      break;
    case STATE_MACHINE_RETURN: 
      // drive to base, dropoff when you see home base
      grip.wristPos = WRIST_UP;
      grip.fingersOpen = false;
      drivingResult = drivingController.drive(currentPoseOdom, goalPoseOdom,
          0.8);
      vel = drivingResult.vel;
      if(homeVisible) {
        if(carryingCube) {
          dropoffController.reset();
          nextState = STATE_MACHINE_DROPOFF;
        } else {
          setGoalPose(searchController.getNextGoal());
          nextState = STATE_MACHINE_SEARCH;
        }
      } else if(!carryingCube && blockVisible) {
        pickUpController.reset();
        nextState = STATE_MACHINE_PICKUP;
      } else if(!drivingResult.busy) {
        // done driving, but not home
        findHomeController.reset();
        nextState = STATE_MACHINE_FIND_HOME;
      }
      break;
    case STATE_MACHINE_FIND_HOME: 
      // if I went home but did not find it, search for it 	
      findResult = findHomeController.execute(obstacleDetected, 
          homeVisible);
      vel = findResult.vel;
      if(findResult.result == LOSER_RESULT_SUCCESS) {
        if(carryingCube)  {
          dropoffController.reset();
          nextState = STATE_MACHINE_DROPOFF;
        } else {
          setGoalPose(searchController.getNextGoal());
          nextState = STATE_MACHINE_SEARCH;
        }
      } else if(!carryingCube && blockVisible) {
        pickUpController.reset();
        nextState = STATE_MACHINE_PICKUP;
      } else if(findResult.result == LOSER_RESULT_FAILED) {
        setGoalPose(0,0);
        nextState = STATE_MACHINE_RETURN;
      }
      break;
    case STATE_MACHINE_DROPOFF: 
      // move to center of base, drop cube, back up, then continue search
      dropoffResult = dropoffController.execute(tagDetectionArray);
      vel = dropoffResult.vel;
      grip = dropoffResult.grip;
      // TODO check for success instead
      if(dropoffResult.result == DROPOFF_RESULT_SUCCESS) {
        carryingCube = false;
        setGoalPose(searchController.getCurrentGoal());
        nextState = STATE_MACHINE_SEARCH;
      } else if(dropoffResult.result != DROPOFF_RESULT_BUSY) {
        setGoalPose(0,0);
        nextState = STATE_MACHINE_RETURN;
      }
      break;
      // pickup cube then return to base
    case STATE_MACHINE_PICKUP: 
      if(stateRunTime > ros::Duration(PICKUP_TIMEOUT)) {
        nextState = STATE_MACHINE_SEARCH;
        break;
      }
      pickupResult = pickUpController.execute(obstacleDetected, 
          tagDetectionArray);
      vel = pickupResult.vel;
      grip = pickupResult.grip;
      if(pickupResult.result == PICKUP_RESULT_SUCCESS) {
        carryingCube = true;
        setGoalPose(0,0);
        nextState = STATE_MACHINE_RETURN;
      } else if(pickupResult.result == PICKUP_RESULT_FAILED) {
        nextState = STATE_MACHINE_SEARCH;
      }
      break;
  } /* end of switch() */
  machineBusy = false;
  // check if home tag visible.  if so, don't go forward to avoid
  // plowing targets off home base.
  /*
     if(homeVisible) {
     switch(state) {
     case STATE_MACHINE_DROPOFF:
     case STATE_MACHINE_PICKUP:
     case STATE_MACHINE_SEARCH:
  //		vel.linear = 0.0;
  }
   */
  sendDriveCommand(vel);
  gripperController->move(grip);
  publishState();
}
void sipi_controller::publishState(void)
{
  // publish state machine string 
  std::ostringstream poses;
  poses <<  fixed << setprecision(1) << 
    " currentPoseA (" <<currentPoseArena.x<<","
    << currentPoseArena.y <<", "<<currentPoseArena.theta<<") " <<
    " O (" <<currentPoseOdom.x<<","
    <<currentPoseOdom.y << ","<<currentPoseOdom.theta<<") " <<
    " goalPoseA (" << goalPoseArena.x <<","
    <<goalPoseArena.y<<","<<goalPoseArena.theta<<") " <<
    " O (" << goalPoseOdom.x <<","
    <<goalPoseOdom.y<<","<<goalPoseOdom.theta<<") ";
  std::ostringstream ss;
  ss <<  fixed << setprecision(1) ;
  switch(state) {
    case STATE_MACHINE_MANUAL: 
      ss << "MANUAL: " << 
        std::setprecision(1) << 
        stateRunTime.toSec() << " Mode= " << currentMode
        << poses.str();
      break;
    case STATE_MACHINE_INIT: 
      ss << "INIT: " << 
        std::setprecision(1) << 
        stateRunTime << 
        " Number Of Rovers = " << numberOfRovers <<
        poses.str();
      break;
    case STATE_MACHINE_SEARCH: 
      ss << "SEARCH: " << 
        std::setprecision(1) << 
        stateRunTime <<
        poses.str() << 
        " linear= "<< vel.linear <<
        " angular= "<< vel.yawError <<
        " dist= "<< drivingResult.distToGoal <<
        " headingToGoal= "<< drivingResult.headingToGoal <<
        " errorYaw= " << drivingResult.errorYaw;
      break;
    case STATE_MACHINE_OBSTACLE: 
      ss << "OBSTACLE: " << 
        std::setprecision(1) << 
        stateRunTime << " : " << obstacleResult.status;
      break;
    case STATE_MACHINE_AVOID_HOME: 
      ss << "AVOID: " << 
        std::setprecision(1) << 
        stateRunTime << " : " << avoidResult.status;
      break;
    case STATE_MACHINE_FIND_HOME: 
      ss << "FIND_HOME: " << 
        std::setprecision(1) << 
        stateRunTime << findResult.status;
      break;
    case STATE_MACHINE_RETURN: 
      ss << "RETURN: " << 
        std::setprecision(1) << 
        stateRunTime << poses.str() ;
      break;
    case STATE_MACHINE_PICKUP: 
      ss << "PICKUP: " << 
        std::setprecision(1) << 
        stateRunTime << " : "<<pickupResult.status;
      break;
    case STATE_MACHINE_DROPOFF: 
      ss << "DROPOFF: " << 
        std::setprecision(1) << 
        stateRunTime << " : "<< dropoffResult.status;
      break;
  } 
  ss << " Carrying: "<< carryingCube;
  std_msgs::String stateMachineMsg;
  stateMachineMsg.data = ss.str();
  stateMachinePublish.publish(stateMachineMsg);
}

void sipi_controller::sendDriveCommand(CDriveCmd vel)
{
  velocity.linear.x = vel.linear;
  velocity.angular.z = vel.yawError;

  // publish the drive commands
  driveControlPublish.publish(velocity);
}
void sipi_controller::sendDriveCommand(float linear, float yawError)
{
  velocity.linear.x = linear;
  velocity.angular.z = yawError;

  // publish the drive commands
  driveControlPublish.publish(velocity);
}
// find out how many rovers are connected
// get a list of all of the topics from the master
// count how many are called "status"
int sipi_controller::countRovers(void)
{
  int count = 0;
  ros::master::V_TopicInfo master_topics;
  ros::master::getTopics(master_topics);

  for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; 
      it != master_topics.end(); it++)
  {
    ros::master::TopicInfo info = *it;
    std::size_t found = info.name.find("/status");
    if (found!=std::string::npos)
    {
      count++;
    }
  }
  return count;
}

/*************************
 * ROS CALLBACK HANDLERS *
 *************************/

void sipi_controller::targetHandler(
    const apriltags_ros::AprilTagDetectionArray::ConstPtr& message) 
{
  // store the detection data
  tagDetectionArray = *message;
}

void sipi_controller::modeHandler(const std_msgs::UInt8::ConstPtr& message) {
  currentMode = message->data;
}

void sipi_controller::obstacleHandler(const std_msgs::UInt8::ConstPtr& message) {
  if(machineBusy) {
    ROS_DEBUG("obstacleHandler called while machineBusy");
    return;
  }
  obstacleDetected = message->data;
}
void sipi_controller::joyCmdHandler(const sensor_msgs::Joy::ConstPtr& message) {
  if (currentMode == 0 || currentMode == 1) {
    sendDriveCommand(abs(message->axes[4]) >= 0.1 ? message->axes[4] : 0, abs(message->axes[3]) >= 0.1 ? message->axes[3] : 0);
  }
}

void sipi_controller::publishStatusTimerEventHandler(const ros::TimerEvent&) {
  std_msgs::String msg;
  msg.data = "online-SIPI";
  status_publisher.publish(msg);
}

void sipi_controller::targetDetectedReset(const ros::TimerEvent& event) {
  targetDetected = false;
  //moveGripper(true, true);
  //TODO what is this doing????
}
void sipi_controller::sigintEventHandler(int sig) {
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

void sipi_controller::publishHeartBeatTimerEventHandler(const ros::TimerEvent&) {
  std_msgs::String msg;
  msg.data = "";
  heartbeatPublisher.publish(msg);
}
// set a new goal.  Input is 2D pose of goal in arena frame
// transforms it to Odom using gps and visual tf
void sipi_controller::setGoalPose(geometry_msgs::Pose2D pose)
{
  goalPoseArena = pose;
  goalPoseOdom = localization->poseArenaToOdom(pose);
}
void sipi_controller::setGoalPose(double x, double y)
{
  goalPoseArena.x = x;
  goalPoseArena.y = y;
  goalPoseOdom = localization->poseArenaToOdom(goalPoseArena);
}
