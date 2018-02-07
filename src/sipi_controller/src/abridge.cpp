#include <ros/ros.h>

//ROS libraries
#include <tf/transform_datatypes.h>

//ROS messages
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/UInt8.h>

//Package include
#include "sipi_controller/usbSerial.h"

// maximum +/- limits for velocity commands
#define MAX_LIN_VEL_CMD 0.3  // m/s
#define MAX_ANG_VEL_CMD 1.0  // rad/s
// maximum safe value to write to motor controller
#define MAX_MOTOR_CMD 120

using namespace std;

//aBridge functions
// theses are the topics that need to communcate through the arduino
// give drive commands.  only listens to linear:x (m/s) and angular:z (rad/s)
void driveCommandHandler(const geometry_msgs::Twist::ConstPtr& message);
// angle in radians for finger joint
void fingerAngleHandler(const std_msgs::Float32::ConstPtr& angle);
// angle in radians for wrist joint
void wristAngleHandler(const std_msgs::Float32::ConstPtr& angle);

// timer to trigger read of serial data from arduino
void serialActivityTimer(const ros::TimerEvent& e);
// helper function to publish all of the ROS topics from arduino data
void publishRosTopics();
//parse the serial data from the arduino into data structures
void parseData(string data);

std::string getHumanFriendlyTime();

//Globals
geometry_msgs::QuaternionStamped fingerAngle;
geometry_msgs::QuaternionStamped wristAngle;
geometry_msgs::Twist cmd_vel_msg;
sensor_msgs::Imu imu;
nav_msgs::Odometry odom;
sensor_msgs::Range sonarLeft;
sensor_msgs::Range sonarCenter;
sensor_msgs::Range sonarRight;
USBSerial usb;
const int baud = 115200;
char dataCmd[] = "d\n";
char moveCmd[16];
char host[128];
const float deltaTime = 0.1; //abridge's update interval
int currentMode = 0;
string publishedName;
float fingerAngle_cmd = 0.0;
float wristAngle_cmd = 0.0;
int motor_left = 0;
int motor_right = 0;
float Kp = 100;
float Kp_turn = 10;
float Ki = 40.0;
float Ki_turn = 20.0;
float int_err_vx = 0.0;
float int_err_vz = 0.0;

float heartbeat_publish_interval = 2;

//Publishers for ROS data from the Arduino
ros::Publisher fingerAnglePublish;
ros::Publisher wristAnglePublish;
ros::Publisher imuPublish;
ros::Publisher odomPublish;
ros::Publisher sonarLeftPublish;
ros::Publisher sonarCenterPublish;
ros::Publisher sonarRightPublish;
ros::Publisher infoLogPublisher;
ros::Publisher heartbeatPublisher;

//Subscribers
ros::Subscriber driveControlSubscriber;
ros::Subscriber fingerAngleSubscriber;
ros::Subscriber wristAngleSubscriber;
ros::Subscriber modeSubscriber;

//Timers
ros::Timer publishTimer;
ros::Timer publish_heartbeat_timer;

//Callback handlers
void publishHeartBeatTimerEventHandler(const ros::TimerEvent& event);
void modeHandler(const std_msgs::UInt8::ConstPtr& message);

int main(int argc, char **argv) {

  gethostname(host, sizeof (host));
  string hostname(host);
  ros::init(argc, argv, (hostname + "_ABRIDGE"));

  ros::NodeHandle param("~");
  string devicePath;
  param.param("device", devicePath, string("/dev/ttyUSB0"));
/*
  param.param("Kp", Kp, 200.0);
  param.param("Kp_turn", Kp_turn, 200.0);
  param.param("Ki", Ki, 1.0);
*/
  usb.openUSBPort(devicePath, baud);

  sleep(5);

  ros::NodeHandle aNH;

  if (argc >= 2) {
    publishedName = argv[1];
    ROS_INFO_STREAM(publishedName << ": ABridge module started." << endl);
  } else {
    publishedName = hostname;
    ROS_INFO_STREAM("abridge: No Name Selected. Default is: " << 
      publishedName << endl);
  }

  fingerAnglePublish = aNH.advertise<geometry_msgs::QuaternionStamped>(
    (publishedName + "/fingerAngle/prev_cmd"), 10);
  wristAnglePublish = aNH.advertise<geometry_msgs::QuaternionStamped>(
    (publishedName + "/wristAngle/prev_cmd"), 10);
  imuPublish = aNH.advertise<sensor_msgs::Imu>((publishedName + "/imu"), 10);
  odomPublish = aNH.advertise<nav_msgs::Odometry>(
    (publishedName + "/odom"), 10);
  sonarLeftPublish = aNH.advertise<sensor_msgs::Range>(
    (publishedName + "/sonarLeft"), 10);
  sonarCenterPublish = aNH.advertise<sensor_msgs::Range>(
    (publishedName + "/sonarCenter"), 10);
  sonarRightPublish = aNH.advertise<sensor_msgs::Range>(
    (publishedName + "/sonarRight"), 10);
  infoLogPublisher = aNH.advertise<std_msgs::String>("/infoLog", 1, true);
  heartbeatPublisher = aNH.advertise<std_msgs::String>(
    (publishedName + "/abridge/heartbeat"), 1, true);

  driveControlSubscriber = aNH.subscribe(
    (publishedName + "/driveControl"), 10, driveCommandHandler);
  fingerAngleSubscriber = aNH.subscribe(
    (publishedName + "/fingerAngle/cmd"), 1, fingerAngleHandler);
  wristAngleSubscriber = aNH.subscribe(
    (publishedName + "/wristAngle/cmd"), 1, wristAngleHandler);
  modeSubscriber = aNH.subscribe(
    (publishedName + "/mode"), 1, modeHandler);


  publishTimer = aNH.createTimer(ros::Duration(deltaTime), serialActivityTimer);
  publish_heartbeat_timer = aNH.createTimer(
    ros::Duration(heartbeat_publish_interval), 
    publishHeartBeatTimerEventHandler);

  imu.header.frame_id = publishedName+"/base_link";

  odom.header.frame_id = publishedName+"/odom";
  odom.child_frame_id = publishedName+"/base_link";
  odom.pose.covariance[0] = 1000; // position x
  odom.pose.covariance[7] = 1000; // position y
  odom.pose.covariance[14] = 1000; // position z
  odom.pose.covariance[21] = 1000; // orientation x
  odom.pose.covariance[28] = 1000; // orientation y
  odom.pose.covariance[35] = 1000; // orientation z

  odom.twist.covariance[0] = 1; // linear x
  odom.twist.covariance[7] = 1; // linear y
  odom.twist.covariance[14] = 1000; // linear z
  odom.twist.covariance[21] = 1000; // angular x
  odom.twist.covariance[28] = 1000; // angular y
  odom.twist.covariance[35] = 1; // angular z

  ros::spin();

  return EXIT_SUCCESS;
}
float limit(float val, float max) {
  if (val > max) {
    return max;
  } else if (val < -max) {
    return -max;
  } else {
    return val;
  } 
}

//This command handler recives a twist setpoint
void driveCommandHandler(const geometry_msgs::Twist::ConstPtr& message) {
  cmd_vel_msg = *message;
  cmd_vel_msg.linear.x = limit(cmd_vel_msg.linear.x, MAX_LIN_VEL_CMD); 
  cmd_vel_msg.angular.z = limit(cmd_vel_msg.angular.z, MAX_ANG_VEL_CMD); 
}
void calculateMotorCommands(void) {
  // Cap motor commands at 120. Experimentally determined that 
  // high values (tested 180 and 255) can cause 
  // the hardware to fail when the robot moves itself too violently.

  // limit motor commands to safe maximum value
  if (cmd_vel_msg.linear.x == 0 && cmd_vel_msg.angular.z == 0) {
    motor_left = motor_right = 0;
    int_err_vx = int_err_vz = 0.0;
  } else {
    geometry_msgs::Twist err;
    err.linear.x = cmd_vel_msg.linear.x - odom.twist.twist.linear.x;
    err.angular.z = cmd_vel_msg.angular.z - odom.twist.twist.angular.z;
    int_err_vx += err.linear.x; 
    int_err_vz += err.angular.z; 
    int vx = err.linear.x * Kp + int_err_vx * Ki;
    int vz = err.angular.z * Kp_turn + int_err_vz * Ki_turn;;
    vz = limit(vz, 100); 
  
    motor_left = limit(vx - vz, MAX_MOTOR_CMD);
    motor_right = limit(vx + vz, MAX_MOTOR_CMD);
/*
    ROS_INFO_STREAM("vel="<<odom.twist.twist.linear.x<<","
      <<odom.twist.twist.angular.z<< " err:"<<err.linear.x 
      << ","<<err.angular.z<<" int_err_vx= "<<int_err_vx
      <<" vx="<<vx<<" vz="<<vz);
*/
  }
}

void sendToArduino(void) {
  ostringstream os;
  // motor command
  // sprintf(moveCmd, "v,%d,%d\n", motor_left, motor_right); 
  os << "v," << motor_left << "," << motor_right << "\n";
  usb.sendData(os.str().c_str());
 // ROS_INFO_STREAM(os.str());
}

// The finger and wrist handlers receive gripper angle commands in 
// floating point
void fingerAngleHandler(const std_msgs::Float32::ConstPtr& angle) {
  if (angle->data < 0.01) {
    fingerAngle_cmd = 0.0;
  } else {
    fingerAngle_cmd = angle->data;
  }
  //finger_update = true;
}

void wristAngleHandler(const std_msgs::Float32::ConstPtr& angle) {
  if (angle->data < 0.01) {
    wristAngle_cmd = 0.0;
  } else {
    wristAngle_cmd = angle->data;
  }
  //wrist_update = true;
}

void serialActivityTimer(const ros::TimerEvent& e) {
  ostringstream os;
  // finger command
  os << "f," << fingerAngle_cmd << "\n";
  // wrist command
  os << "w," << wristAngle_cmd << "\n";
  os << "d\n";
  usb.sendData(os.str().c_str());
  //ROS_INFO_STREAM(os.str());
  parseData(usb.readData());
  publishRosTopics();
  calculateMotorCommands();
  sendToArduino();
}

void publishRosTopics() {
  fingerAnglePublish.publish(fingerAngle);
  wristAnglePublish.publish(wristAngle);
  imuPublish.publish(imu);
  odomPublish.publish(odom);
  sonarLeftPublish.publish(sonarLeft);
  sonarCenterPublish.publish(sonarCenter);
  sonarRightPublish.publish(sonarRight);
}

void parseData(string str) {
  istringstream oss(str);
  string sentence;

  while (getline(oss, sentence, '\n')) {
    istringstream wss(sentence);
    string word;

    vector<string> dataSet;
    while (getline(wss, word, ',')) {
      dataSet.push_back(word);
    }

    if (dataSet.size() >= 3 && dataSet.at(1) == "1") {

      if (dataSet.at(0) == "GRF") {
        fingerAngle.header.stamp = ros::Time::now();
        fingerAngle.quaternion = tf::createQuaternionMsgFromRollPitchYaw(
          atof(dataSet.at(2).c_str()), 0.0, 0.0);
      }
      else if (dataSet.at(0) == "GRW") {
        wristAngle.header.stamp = ros::Time::now();
        wristAngle.quaternion = tf::createQuaternionMsgFromRollPitchYaw(
          atof(dataSet.at(2).c_str()), 0.0, 0.0);
      }
      else if (dataSet.at(0) == "IMU") {
        imu.header.stamp = ros::Time::now();
        imu.linear_acceleration.x = atof(dataSet.at(2).c_str());
        imu.linear_acceleration.y = 0; //atof(dataSet.at(3).c_str());
        imu.linear_acceleration.z = atof(dataSet.at(4).c_str());
        imu.angular_velocity.x = atof(dataSet.at(5).c_str());
        imu.angular_velocity.y = atof(dataSet.at(6).c_str());
        imu.angular_velocity.z = atof(dataSet.at(7).c_str());
        imu.orientation = tf::createQuaternionMsgFromRollPitchYaw(
          atof(dataSet.at(8).c_str()), atof(dataSet.at(9).c_str()), 
          atof(dataSet.at(10).c_str()));
      }
      else if (dataSet.at(0) == "ODOM") {
        odom.header.stamp = ros::Time::now();
        odom.pose.pose.position.x += atof(dataSet.at(2).c_str()) / 100.0;
        odom.pose.pose.position.y += atof(dataSet.at(3).c_str()) / 100.0;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(
          atof(dataSet.at(4).c_str()));
        odom.twist.twist.linear.x = atof(dataSet.at(5).c_str()) / 100.0;
        odom.twist.twist.linear.y = atof(dataSet.at(6).c_str()) / 100.0;
        odom.twist.twist.angular.z = atof(dataSet.at(7).c_str());
      }
      else if (dataSet.at(0) == "USL") {
        sonarLeft.header.stamp = ros::Time::now();
        sonarLeft.range = atof(dataSet.at(2).c_str()) / 100.0;
      }
      else if (dataSet.at(0) == "USC") {
        sonarCenter.header.stamp = ros::Time::now();
        sonarCenter.range = atof(dataSet.at(2).c_str()) / 100.0;
      }
      else if (dataSet.at(0) == "USR") {
        sonarRight.header.stamp = ros::Time::now();
        sonarRight.range = atof(dataSet.at(2).c_str()) / 100.0;
      }

    }
  }
}

void modeHandler(const std_msgs::UInt8::ConstPtr& message) {
  currentMode = message->data;
}

void publishHeartBeatTimerEventHandler(const ros::TimerEvent&) {
  std_msgs::String msg;
  msg.data = "";
  heartbeatPublisher.publish(msg);
}
