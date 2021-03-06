#include "sipi_controller/sipi_controller.h"

#include <geometry_msgs/Pose2D.h>
//#include "sipi_controller/PoseHelpers.h"
#include <stdlib.h>
#include <time.h>
///////////////////////////////////////////////////////////////////
float f_rand(float range) {
  return (float)(rand()-RAND_MAX/2)/(float)(RAND_MAX/2)*range;
}
sipi_controller::sipi_controller(
    const std::string &name, 
    int argc, char** argv) {

  ros::NodeHandle mNH;
  state = STATE_MACHINE_MANUAL;
  prevState = STATE_MACHINE_MANUAL;
  avoidPrevState = STATE_MACHINE_MANUAL;
  nextState = STATE_MACHINE_MANUAL;
  obstacle_detected = false;
  obstacle_count = 0;
  numberOfRovers = 1; // how many rovers are active
  currentMode = 0;
  carryingCube = false;
  stateMachinePeriod = 0.1; // time between the state machine loop
  status_publish_interval = 1;
  watchdogTimeout = 75;
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
  arena_radius_ = 9.0;

  gripperController = new GripperController(mNH, name);
  localization = new Localization(name, mNH);

  joySubscriber = mNH.subscribe((name + "/joystick"), 10, 
      &sipi_controller::joyCmdHandler, this);
  modeSubscriber = mNH.subscribe((name + "/mode"), 1, 
      &sipi_controller::modeHandler, this);
  targetSubscriber = mNH.subscribe((name + "/targets"), 10, 
      &sipi_controller::targetHandler, this);
  sonarLeftSubscriber = mNH.subscribe((name + "/sonarLeft"), 10, 
      &sipi_controller::sonarLeftHandler, this);
  sonarCenterSubscriber = mNH.subscribe((name + "/sonarCenter"), 10, 
      &sipi_controller::sonarCenterHandler, this);
  sonarRightSubscriber = mNH.subscribe((name + "/sonarRight"), 10, 
      &sipi_controller::sonarRightHandler, this);

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
  srand (static_cast <unsigned> (time(0)));
  cout << "RND:"<<f_rand(2) <<","<< f_rand(5) << ","<<f_rand(10);
}
void sipi_controller::stateMachine(const ros::TimerEvent&) {
  // publish state machine string 
  std::ostringstream poses;
  poses <<  fixed << setprecision(1) << 
    " currentA (" <<currentPoseArena.x<<","
    << currentPoseArena.y <<", "<<currentPoseArena.theta<<") " <<
    " O (" <<currentPoseOdom.x<<","
    <<currentPoseOdom.y << ","<<currentPoseOdom.theta<<") ";
  std::ostringstream goal_poses;
  goal_poses << " goalA (" << goalPoseArena.x <<","
    <<goalPoseArena.y<<","<<goalPoseArena.theta<<") " <<
    " O (" << goalPoseOdom.x <<","
    <<goalPoseOdom.y<<","<<goalPoseOdom.theta<<") ";
  std::ostringstream status_stream;
  status_stream <<  fixed << setprecision(1) << 
    stateRunTime.toSec() << " " << carryingCube << " ";

  // translate the tag detections into vectors of tag locations
  std::vector<geometry_msgs::Pose2D> home_tags;
  std::vector<geometry_msgs::Pose2D> cube_tags;
  getTagsVector(tagDetectionArray, home_tags, 256);
  getTagsVector(tagDetectionArray, cube_tags, 0);
  homeVisible = !home_tags.empty();
  blockVisible = !cube_tags.empty();
  // first find where we are
  // to ignore GPS and visual, set both to Odom
  localization->execute(home_tags);
  currentPoseOdom = localization->getPoseOdom();
  currentPoseArena = localization->getPoseArena();
  // this allows the target to dissappear briefly but not consider it lost
  // at first (reset) homeSeen is false. but then it only becomes false
  // again if the target is missed multiple times
  if(homeVisible) {
    missedHomeCount = 0;
    homeSeen = true;
  } else {
    missedHomeCount++;
    if(missedHomeCount > 10) {
      homeSeen = false;
    }
  }
  obstacle_detected = Obstacle::obstacleDetected(ultrasound);
  if(obstacle_detected) {
    obstacle_count++;
  } else {
    obstacle_count = 0;
  }
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
    setGoalPoseArena(0,0);
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
      // print out the home_tag array
      status_stream << setprecision(3);
      if (!home_tags.empty()) {
        status_stream << " Home:";
        for( auto t : home_tags) 
          status_stream << "("<<t.x<<","<<t.y<<","<<t.theta<<")" ;
      }
      if (!cube_tags.empty()) {
        status_stream << " Cubes:";
        for( auto t : cube_tags) 
          status_stream << "("<<t.x<<","<<t.y<<","<<t.theta<<")" ;
      }
      status_stream << "OBS "<< obstacle_detected <<","<<obstacle_count << "("<<ultrasound.x<<","<<ultrasound.y<<","<<ultrasound.z<<")";
      status_stream << setprecision(1);

      break;
    case STATE_MACHINE_INIT:
      if (stateRunTime > ros::Duration(1)) {
        init = true;
        numberOfRovers = countRovers();
        if(numberOfRovers > 3) {
          arena_radius_ = 9.0;
        } else {
          arena_radius_ = 5.0;
        }
        //currentPoseArena = localization->getPoseArena();
        searchController.createPattern(currentPoseArena, numberOfRovers);
        searchController.reset();
        setGoalPoseArena(searchController.getNextGoal());
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
      if(stateRunTime > ros::Duration(60)) {
        ROS_WARN("Search timeout, going to next pose");
        setGoalPoseArena(0,0);
        nextState = STATE_MACHINE_RETURN;
      }
      drivingResult = drivingController.drive(currentPoseOdom, 
          goalPoseOdom, 0.2);
      cmd_vel_ = drivingResult.cmd_vel;
      if(blockVisible && !homeSeen && !pickUpController.ignore_cubes()) {
        pickUpController.reset();  
        nextState = STATE_MACHINE_PICKUP;
      } else if(!drivingResult.busy) {   // arrived at goal
        setGoalPoseArena(0,0);
        nextState = STATE_MACHINE_RETURN;
      }
      status_stream << "SEARCH: " << 
        poses.str() << goal_poses.str() <<
        " dist= "<< drivingResult.distToGoal <<
        " headingToGoal= "<< drivingResult.headingToGoal <<
        " errorYaw= " << drivingResult.errorYaw;
      break;
    case STATE_MACHINE_AVOID_HOME: 
      avoidResult = avoidController.execute(home_tags);
      cmd_vel_ = avoidResult.cmd_vel;
      if(avoidResult.result != AvoidHome::ResultCode::BUSY) {
        nextState = avoidPrevState;
      }
      status_stream << "AVOID: " << 
        std::setprecision(1) << 
        stateRunTime << " : " << avoidResult.status;
      break;
      // avoid an obstacle then continue
    case STATE_MACHINE_OBSTACLE: 
      if(blockVisible && !carryingCube && 
          !pickUpController.ignore_cubes()) {
        pickUpController.reset();
        nextState = STATE_MACHINE_PICKUP;
      } else {
        obstacleResult = obstacleController.execute(ultrasound, 
            currentPoseOdom.theta);
        cmd_vel_ = obstacleResult.cmd_vel;
        if(obstacleResult.result == Obstacle::ResultCode::SUCCESS) {
          pickUpController.reset();
          nextState = prevState;
        } else if(obstacleResult.result == 
            Obstacle::ResultCode::FAILED) {
          pickUpController.reset();
          findHomeController.reset();
          nextState = STATE_MACHINE_FIND_HOME;
        }
      }
      status_stream << "OBSTACLE: " << 
        std::setprecision(1) << 
        stateRunTime << " : " << obstacleResult.status;
      break;
    case STATE_MACHINE_RETURN: 
      {
        // drive to base, dropoff when you see home base
        grip.wristPos = WRIST_UP;
        grip.fingersOpen = false;
        float return_speed;
        if(carryingCube) {
          return_speed = 0.3;
        } else {
          return_speed = 0.2;
        }
        drivingResult = drivingController.drive(currentPoseOdom, goalPoseOdom,
            return_speed);
        cmd_vel_ = drivingResult.cmd_vel;
        if(homeVisible) {
          if(carryingCube) {
            dropoffController.reset();
            nextState = STATE_MACHINE_DROPOFF;
          } else {
            setGoalPoseArena(searchController.getNextGoal());
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
      }
    case STATE_MACHINE_FIND_HOME: 
      // if I went home but did not find it, search for it 	
      findResult = findHomeController.execute(obstacle_detected, 
          homeVisible);
      cmd_vel_ = findResult.cmd_vel;
      if(findResult.result == FindHome::ResultCode::SUCCESS) {
        if(carryingCube)  {
          dropoffController.reset();
          nextState = STATE_MACHINE_DROPOFF;
        } else {
          setGoalPoseArena(searchController.getNextGoal());
          nextState = STATE_MACHINE_SEARCH;
        }
      } else if(!carryingCube && blockVisible) {
        pickUpController.reset();
        nextState = STATE_MACHINE_PICKUP;
      } else if(findResult.result == FindHome::ResultCode::FAILED) {
        // see where GPS thinks we are and then go home based on that
        geometry_msgs::Pose2D zeroPose;
        geometry_msgs::Pose2D p = localization->poseUTMToOdom(zeroPose);
        float angle = f_rand(M_PI); 
        p.x = p.x + 1.0*sin(angle);
        p.y = p.y + 1.0*cos(angle);
        setGoalPoseOdom(p);
        cout << "Goal is now " <<goalPoseOdom;
        nextState = STATE_MACHINE_RETURN;
      }
      status_stream << "FIND_HOME: " << 
        std::setprecision(1) << 
        stateRunTime << findResult.status;
      break;
    case STATE_MACHINE_DROPOFF: 
      // move to center of base, drop cube, back up, then continue search
      dropoffResult = dropoffController.execute(home_tags);
      cmd_vel_ = dropoffResult.cmd_vel;
      grip = dropoffResult.grip;
      // TODO check for success instead
      if(dropoffResult.result == Dropoff::ResultCode::SUCCESS) {
        carryingCube = false;
        setGoalPoseArena(searchController.getCurrentGoal());
        nextState = STATE_MACHINE_SEARCH;
      } else if(dropoffResult.result != Dropoff::ResultCode::BUSY) {
        setGoalPoseArena(0,0);
        nextState = STATE_MACHINE_RETURN;
      }
      status_stream << "DROPOFF: " << 
        std::setprecision(1) << 
        stateRunTime << " : "<< dropoffResult.status;
      break;
    case STATE_MACHINE_PICKUP: 
      // pickup cube 
      if(stateRunTime > ros::Duration(45)) {
        ROS_WARN("PICKUP_TIMEOUT exceeded in sipi_controller");
        nextState = STATE_MACHINE_SEARCH;
        break;
      }
      pickupResult = pickUpController.execute(obstacle_detected, 
          cube_tags);
      cmd_vel_ = pickupResult.cmd_vel;
      grip = pickupResult.grip;
      if (pickupResult.result == PickupController::ResultCode::SUCCESS) {
        carryingCube = true;
        setGoalPoseArena(0,0);
        nextState = STATE_MACHINE_RETURN;
      } else if (pickupResult.result == PickupController::ResultCode::FAILED) {
        nextState = STATE_MACHINE_SEARCH;
      }
      status_stream << "PICKUP: " << pickupResult.status;
      break;
  } /* end of switch() */

  // check if home tag visible.  if so, don't go forward to avoid
  // plowing targets off home base.
#if 1
  if(homeVisible) {
    switch(state) {
      case STATE_MACHINE_SEARCH:
        if(cmd_vel_.linear.x > 0) {
          avoidPrevState = state;
          avoidController.reset();
          cmd_vel_.linear.x = 0.0;
          nextState = STATE_MACHINE_AVOID_HOME;
        }
        break;
      case STATE_MACHINE_PICKUP:
        status_stream << " Home visible while pickup, aborting! ";
        nextState = STATE_MACHINE_AVOID_HOME;
        cmd_vel_.linear.x = 0.0;
        break;
      case STATE_MACHINE_OBSTACLE:
        status_stream << " Home visible while avoiding obstacle, aborting! ";
        cmd_vel_.linear.x = 0.0;
        nextState = STATE_MACHINE_AVOID_HOME;
        break;
        //		cmd_vel_.linear.x = 0.0;
    }
  }
#endif
#if 1
  if(obstacle_count > 3) {
    geometry_msgs::Pose2D poseUTM = localization->getPoseUTM();
    float distance = sqrt(poseUTM.x*poseUTM.x+poseUTM.y*poseUTM.y);
    status_stream << " dist " << distance << " arena_rad " << arena_radius_;
    if(distance > arena_radius_ && state == STATE_MACHINE_SEARCH) {
      setGoalPoseArena(0,0);
      nextState = STATE_MACHINE_RETURN;
    } else if ((state != STATE_MACHINE_OBSTACLE) &&
        (state != STATE_MACHINE_MANUAL) && (state != STATE_MACHINE_PICKUP)
        && (state != STATE_MACHINE_DROPOFF && cmd_vel_.linear.x != 0.0)) {
      // remember what we were doing
      prevState = state;
      obstacleController.reset();
      nextState = STATE_MACHINE_OBSTACLE;
    }
  }
#endif
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

void sipi_controller::sonarLeftHandler(const sensor_msgs::Range::ConstPtr& message) {
  ultrasound.x = message->range;
}
void sipi_controller::sonarCenterHandler(const sensor_msgs::Range::ConstPtr& message) {
  ultrasound.y = message->range;
}
void sipi_controller::sonarRightHandler(const sensor_msgs::Range::ConstPtr& message) {
  ultrasound.z = message->range;
}

void sipi_controller::joyCmdHandler(
    const sensor_msgs::Joy::ConstPtr& message) {
  lastJoyCmd = *message;
}

void sipi_controller::publishStatusTimerEventHandler(const ros::TimerEvent&) {
  std_msgs::String msg;
  msg.data = "SIPI";
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
void sipi_controller::setGoalPoseArena(geometry_msgs::Pose2D pose) {
  goalPoseArena = pose;
  goalPoseOdom = localization->poseArenaToOdom(pose);
}
void sipi_controller::setGoalPoseOdom(geometry_msgs::Pose2D pose) {
  goalPoseArena = localization->poseOdomToArena(pose);
  goalPoseOdom = pose;
}

void sipi_controller::setGoalPoseArena(double x, double y) {
  goalPoseArena.x = x;
  goalPoseArena.y = y;
  goalPoseOdom = localization->poseArenaToOdom(goalPoseArena);
}
