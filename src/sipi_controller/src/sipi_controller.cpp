#include "sipi_controller/sipi_controller.h"

///////////////////////////////////////////////////////////////////
sipi_controller::sipi_controller(
    const std::string &name, 
    int argc, char** argv) {

  ros::NodeHandle mNH;
  state = STATE_MACHINE_MANUAL;
  prevState = STATE_MACHINE_MANUAL;
  avoidPrevState = STATE_MACHINE_MANUAL;
  nextState = STATE_MACHINE_MANUAL;
  obstacleDetected = 0;
  obstacleCount = 0;
  numberOfRovers = 1; // how many rovers are active
  currentMode = 0;
  carryingCube = false;
  stateMachinePeriod = 0.1; // time between the state machine loop
  status_publish_interval = 1;
  watchdogTimeout = 60;
  targetDetected = false;
  targetCollected = false;
  homeVisible = false;
  blockVisible = false;
  missedHomeCount = 0;
  homeSeen = false; // allows it to be missed a few times before going false
  // is state machine processing?  if so hold off on asyn updates from topics
  machineBusy = true;
  // Set true when we are insie the center circle and we need to drop the block,
  // back out, and reset the boolean cascade.
  reachedCollectionPoint = false;
  // used for calling code once but not in main
  init = false;
  avoidingObstacle = false;

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
  // publish state machine string 
  std::ostringstream poses;
  poses <<  fixed << setprecision(1) << 
    " currentPoseA (" <<currentPoseArena.x<<","
    << currentPoseArena.y <<", "<<currentPoseArena.theta<<") " <<
    " O (" <<currentPoseOdom.x<<","
    <<currentPoseOdom.y << ","<<currentPoseOdom.theta<<") ";
  std::ostringstream goal_poses;
  goal_poses << " goalPoseA (" << goalPoseArena.x <<","
    <<goalPoseArena.y<<","<<goalPoseArena.theta<<") " <<
    " O (" << goalPoseOdom.x <<","
    <<goalPoseOdom.y<<","<<goalPoseOdom.theta<<") ";
  std::ostringstream status_stream;
  status_stream <<  fixed << setprecision(1) << 
    stateRunTime.toSec() << " " << carryingCube << " ";

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
    if(drivingResult.cmd_vel.linear.x > 0) {
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
  cmd_vel_.linear.x = cmd_vel_.angular.z = 0.0;
  CGripCmd grip;

  /***
    This is the watchdog timer.  if any state hangs for more than 
    the watchdog time, then go home and start over
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
      if(lastJoyCmd.axes.size() >= 4 ) {
        cmd_vel_.linear.x = abs(lastJoyCmd.axes[4]) >= 0.1 ?
          lastJoyCmd.axes[4] : 0; 
        cmd_vel_.angular.z = abs(lastJoyCmd.axes[3]) >= 0.1 ?
          lastJoyCmd.axes[3] : 0;
      }
      if (currentMode == 2 || currentMode == 3) {
        nextState = STATE_MACHINE_INIT;
      }
      status_stream << "MANUAL: " << " Mode= " << currentMode << poses.str();
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
      status_stream << "INIT: " << 
        std::setprecision(1) << 
        stateRunTime << 
        " Number Of Rovers = " << numberOfRovers <<
        poses.str();
      break;
    case STATE_MACHINE_SEARCH: 
      // go through search pattern goals
      if(stateRunTime > ros::Duration(30)) {
        ROS_WARN("Search timeout, going to next pose");
        setGoalPose(searchController.getNextGoal());
      }
      drivingResult = drivingController.drive(currentPoseOdom, 
          goalPoseOdom, 0.2);
      cmd_vel_ = drivingResult.cmd_vel;
      if(blockVisible && !homeSeen) {
        pickUpController.reset();  
        nextState = STATE_MACHINE_PICKUP;
      } else if(!drivingResult.busy) {   // arrived at goal
        setGoalPose(0,0);
        nextState = STATE_MACHINE_RETURN;
      }
      status_stream << "SEARCH: " << 
        poses.str() << goal_poses.str() <<
        " dist= "<< drivingResult.distToGoal <<
        " headingToGoal= "<< drivingResult.headingToGoal <<
        " errorYaw= " << drivingResult.errorYaw;
      break;
    case STATE_MACHINE_AVOID_HOME: 
      avoidResult = avoidController.execute(homeSeen);
      cmd_vel_ = avoidResult.cmd_vel;
      if(avoidResult.result != AVOID_RESULT_BUSY) {
        nextState = avoidPrevState;
      }
      status_stream << "AVOID: " << 
        std::setprecision(1) << 
        stateRunTime << " : " << avoidResult.status;
      break;
      // avoid an obstacle then continue
    case STATE_MACHINE_OBSTACLE: 
      obstacleResult = obstacleController.execute(obstacleDetected);
      cmd_vel_ = obstacleResult.cmd_vel;
      if(obstacleResult.result == OBS_RESULT_SUCCESS) {
        nextState = prevState;
      } else if(obstacleResult.result == OBS_RESULT_FAILED) {
        findHomeController.reset();
        nextState = STATE_MACHINE_FIND_HOME;
      }
      status_stream << "OBSTACLE: " << 
        std::setprecision(1) << 
        stateRunTime << " : " << obstacleResult.status;
      break;
    case STATE_MACHINE_RETURN: 
      // drive to base, dropoff when you see home base
      grip.wristPos = WRIST_UP;
      grip.fingersOpen = false;
      drivingResult = drivingController.drive(currentPoseOdom, goalPoseOdom,
          0.8);
      cmd_vel_ = drivingResult.cmd_vel;
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
      status_stream << "RETURN: " << 
        poses.str() << goal_poses.str() <<
        " dist= "<< drivingResult.distToGoal <<
        " headingToGoal= "<< drivingResult.headingToGoal <<
        " errorYaw= " << drivingResult.errorYaw;
      break;
    case STATE_MACHINE_FIND_HOME: 
      // if I went home but did not find it, search for it 	
      findResult = findHomeController.execute(obstacleDetected, 
          homeVisible);
      cmd_vel_ = findResult.cmd_vel;
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
      status_stream << "FIND_HOME: " << 
        std::setprecision(1) << 
        stateRunTime << findResult.status;
      break;
    case STATE_MACHINE_DROPOFF: 
      // move to center of base, drop cube, back up, then continue search
      dropoffResult = dropoffController.execute(tagDetectionArray);
      cmd_vel_ = dropoffResult.cmd_vel;
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
      status_stream << "DROPOFF: " << 
        std::setprecision(1) << 
        stateRunTime << " : "<< dropoffResult.status;
      break;
    case STATE_MACHINE_PICKUP: 
      // pickup cube 
      if(stateRunTime > ros::Duration(PICKUP_TIMEOUT)) {
        nextState = STATE_MACHINE_SEARCH;
        break;
      }
      pickupResult = pickUpController.execute(obstacleDetected, 
          tagDetectionArray);
      cmd_vel_ = pickupResult.cmd_vel;
      grip = pickupResult.grip;
      if (pickupResult.result == PickupController::ResultCode::SUCCESS) {
        carryingCube = true;
        setGoalPose(0,0);
        nextState = STATE_MACHINE_RETURN;
      } else if (pickupResult.result == PickupController::ResultCode::FAILED) {
        nextState = STATE_MACHINE_SEARCH;
      }
      status_stream << "PICKUP: " << pickupResult.status;
      break;
  } /* end of switch() */
  // check if home tag visible.  if so, don't go forward to avoid
  // plowing targets off home base.
  /*
     if(homeVisible) {
     switch(state) {
     case STATE_MACHINE_DROPOFF:
     case STATE_MACHINE_PICKUP:
     case STATE_MACHINE_SEARCH:
  //		cmd_vel_.linear.x = 0.0;
  }
   */
  driveControlPublish.publish(cmd_vel_);
  gripperController->move(grip);
  status_stream << " cmd_vel_=" << cmd_vel_.linear.x <<
    "," <<cmd_vel_.angular.z;
  std_msgs::String stateMachineMsg;
  stateMachineMsg.data = status_stream.str();
  stateMachinePublish.publish(stateMachineMsg);
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
      it != master_topics.end(); it++) {
    ros::master::TopicInfo info = *it;
    std::size_t found = info.name.find("/status");
    if (found != std::string::npos) {
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

void sipi_controller::joyCmdHandler(
    const sensor_msgs::Joy::ConstPtr& message) {
  lastJoyCmd = *message;
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
void sipi_controller::setGoalPose(geometry_msgs::Pose2D pose) {
  goalPoseArena = pose;
  goalPoseOdom = localization->poseArenaToOdom(pose);
}

void sipi_controller::setGoalPose(double x, double y) {
  goalPoseArena.x = x;
  goalPoseArena.y = y;
  goalPoseOdom = localization->poseArenaToOdom(goalPoseArena);
}
