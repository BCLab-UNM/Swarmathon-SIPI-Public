#include "sipi_controller/Localization.h"
/***
  Localization class is respponsible for calculating the pose of
  the rover in various frames.

  odom frame:  this is the odometry referenced from
  the starting point of the rover.  It is continuous
  and used for driving control

  EKF frame:   this is the output of the Kalman filter combining
  the odom data and the navsat.  Depending on the setting for
  "use_gps" in the launch file it may be the same as odom

  UTM frame:   this was added to get around the drift of the EKF frame
  It is purely the gps based pose, offset to zero when home is 
  sighted.

  arena frame: this is the location of the rover with respect to the home
  base.
 */

geometry_msgs::Pose2D operator+(const geometry_msgs::Pose2D &p1, 
  const geometry_msgs::Pose2D &p2) {
  geometry_msgs::Pose2D p;
  p.x = p1.x+p2.x;
  p.y = p1.y+p2.y;
  p.theta = p1.theta+p2.theta;
  return p;
}
geometry_msgs::Pose2D operator-(const geometry_msgs::Pose2D &p1, 
  const geometry_msgs::Pose2D &p2) {
  geometry_msgs::Pose2D p;
  p.x = p1.x-p2.x;
  p.y = p1.y-p2.y;
  p.theta = p1.theta-p2.theta;
  return p;
}
geometry_msgs::Pose2D operator-(const geometry_msgs::Pose2D &p1) {
  geometry_msgs::Pose2D p;
  p.x = -p1.x;
  p.y = -p1.y;
  p.theta = -p1.theta;
  return p;
}
std::ostream &operator<<(std::ostream &os, const geometry_msgs::Pose2D &p) {
  return os <<"("<<p.x<<","<<p.y<<","<<p.theta<< ")";
}

void Localization::execute(const std::vector<geometry_msgs::Pose2D> &home_tags){
  geometry_msgs::Pose2D p_avg;
  if(utm_offset.x==0) utm_offset = -poseUTM_raw;
  // find average of the poses of the home tags
  // ignore angle since home base is not uniform
  if (!home_tags.empty()) {
    int cnt = 0;
    for (const auto &p : home_tags) {
      p_avg = p_avg + p;
      cnt++;
    }
    p_avg.x /= (double)cnt;
    p_avg.y /= (double)cnt;
    tParentToChild.setOrigin(tf::Vector3(p_avg.x, p_avg.y, 0));
    pose_visual.x = p_avg.x * cos(poseEKF.theta - M_PI);
    pose_visual.y = p_avg.x * sin(poseEKF.theta - M_PI);
    arena_offset = pose_visual - poseEKF;
    utm_offset = pose_visual - poseUTM_raw;
  }
  poseUTM = poseUTM_raw + utm_offset;
  poseArena = poseOdomToArena(poseOdom);
  tf_pub.sendTransform(tf::StampedTransform(tParentToChild, ros::Time::now(),
        arenaFrame, mapFrame));

  //std::cout << " UTM "<< poseUTM << std::endl;
  std_msgs::String msg;
  std::ostringstream ss;
  geometry_msgs::Pose2D zeroPose;
  ss << std::fixed << std::setprecision(2) << 
    "Arena: " << poseArena <<
    " odom: " << poseOdom <<
    " EKF: " << poseEKF <<
    " offset: " << arena_offset <<
    " avg : " << p_avg <<
    " pose_visual : " << pose_visual << 
    " UTM " << poseUTM <<
    " UTM0 in odom " << poseUTMToOdom(zeroPose) <<
    statusMsg;
  msg.data = ss.str();
  status_publisher.publish(msg);
  statusMsg.clear();
}

Localization::Localization(std::string _name, ros::NodeHandle _nh)
{
  mNH = _nh;
  name = _name;
  tfListener = new tf::TransformListener();
  odometrySubscriber = mNH.subscribe((name + "/odom/filtered"), 10, 
      &Localization::odometryHandler, this);
  odomEKFSubscriber = mNH.subscribe((name + "/odom/ekf"), 10, 
      &Localization::odomEKFHandler, this);
  odomUTMSubscriber = mNH.subscribe((name + "/odom/utm"), 10, 
      &Localization::odomUTMHandler, this);
  status_publisher = mNH.advertise<std_msgs::String>((name + "/loc_status"), 1, true);
  arenaFrame = name+"/arena";
  mapFrame = name+"/map";
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  tParentToChild.setRotation(q);

}
/*** get pose wrt UTM.  only includes the UTM offset */
geometry_msgs::Pose2D Localization::getPoseUTM(void) 
{
  geometry_msgs::Pose2D pose;
  pose.x = poseUTM.x;
  pose.y = poseUTM.y;
  return pose;
}
/*** get pose wrt the odom frame.  used for local driving */
geometry_msgs::Pose2D Localization::getPoseOdom(void) 
{
  return poseOdom;
}
/*** get pose wrt the arena frame.  used for arena based goals */
geometry_msgs::Pose2D Localization::getPoseArena(void) 
{
  return poseArena;
}

// convert a pose in the UTM frame to a pose in the odom frame
geometry_msgs::Pose2D Localization::poseUTMToOdom( 
    geometry_msgs::Pose2D p_utm 
    )
{
  geometry_msgs::Pose2D p;
  p = p_utm - poseUTM + poseOdom;
  return p;
}
// convert a pose in the odom frame to a pose in the arena frame
geometry_msgs::Pose2D Localization::poseOdomToArena( 
    geometry_msgs::Pose2D poseOdom2D
    )
{
  geometry_msgs::Pose2D p;

  p.x = poseEKF.x + arena_offset.x;
  p.y = poseEKF.y + arena_offset.y;
  p.theta = poseOdom2D.theta;
  return p;
}
// convert a pose in the arena frame to a pose in the odom frame
geometry_msgs::Pose2D Localization::poseArenaToOdom( 
    geometry_msgs::Pose2D poseArena2D
    )
{
  geometry_msgs::Pose2D p;
  p.x = poseArena2D.x - arena_offset.x - poseEKF.x + poseOdom.x;
  p.y = poseArena2D.y - arena_offset.y - poseEKF.y + poseOdom.y;
  return p;
}
// transform a 2D pose from one frame to another
geometry_msgs::Pose2D Localization::transformPose( 
    geometry_msgs::Pose2D poseIn2D, std::string from_frame, std::string to_frame
    )
{
  geometry_msgs::PoseStamped poseIn3D = pose2Dto3D(poseIn2D,from_frame);
  geometry_msgs::PoseStamped poseOut;

  try { 
    tfListener->transformPose(to_frame, ros::Time(0), poseIn3D, from_frame, poseOut);
    return pose3Dto2D(poseOut);
  } catch(tf::TransformException& ex) {
    statusMsg = "Transform Failed from " + from_frame + " to " + to_frame;
    ROS_WARN("Error transform point from %s to %s: %s", 
        from_frame.c_str(), to_frame.c_str(), ex.what());
    geometry_msgs::Pose2D zeroPose;
    zeroPose.x = zeroPose.y = zeroPose.theta = 0.0;
    return zeroPose;
  }
}

geometry_msgs::PoseStamped pose2Dto3D( 
    geometry_msgs::Pose2D &pose2D,
    std::string frame_id
    )
{
  // fill poseArena from pose2D
  geometry_msgs::PoseStamped pose3D;
  pose3D.header.stamp = ros::Time::now();
  pose3D.header.frame_id = frame_id;
  pose3D.pose.orientation = tf::createQuaternionMsgFromYaw(pose2D.theta);
  pose3D.pose.position.x = pose2D.x;
  pose3D.pose.position.y = pose2D.y;
  return pose3D;
}
geometry_msgs::Pose2D pose3Dto2D( 
    geometry_msgs::PoseStamped &pose3D
    )
{
  geometry_msgs::Pose2D pose2D; 
  pose2D.x = pose3D.pose.position.x;
  pose2D.y = pose3D.pose.position.y;
  pose2D.theta = thetaFromQuat(pose3D.pose.orientation);
  return pose2D;
}
//Get theta rotation by converting quaternion orientation to pitch/roll/yaw
double thetaFromQuat(geometry_msgs::Quaternion quat) 
{
  tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}
void Localization::odometryHandler(const nav_msgs::Odometry::ConstPtr& message) {
  poseOdom.x = message->pose.pose.position.x;
  poseOdom.y = message->pose.pose.position.y;
  poseOdom.theta = thetaFromQuat(message->pose.pose.orientation);
}
void Localization::odomEKFHandler(const nav_msgs::Odometry::ConstPtr& message) {
  poseEKF.x = message->pose.pose.position.x;
  poseEKF.y = message->pose.pose.position.y;
  poseEKF.theta = thetaFromQuat(message->pose.pose.orientation);
}
void Localization::odomUTMHandler(const nav_msgs::Odometry::ConstPtr& message) {
  poseUTM_raw.x = message->pose.pose.position.x;
  poseUTM_raw.y = message->pose.pose.position.y;
}
