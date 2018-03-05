#include "sipi_controller/Localization.h"

void Localization::execute(const std::vector<geometry_msgs::Pose2D> &home_tags){
  geometry_msgs::Pose2D p_avg;
  // find average of the poses of the home tags
  // ignore angle since home base is not uniform
  if (!home_tags.empty()) {
    int cnt = 0;
    for (const auto &p : home_tags) {
      p_avg.x += p.x;
      p_avg.y += p.y;
      cnt++;
    }
    p_avg.x /= (double)cnt;
    p_avg.y /= (double)cnt;
    tParentToChild.setOrigin(tf::Vector3(p_avg.x, p_avg.y, 0));
    pose_visual.x = p_avg.x * cos(poseGPS.theta - M_PI);
    pose_visual.y = p_avg.x * sin(poseGPS.theta - M_PI);
    arena_offset.x = pose_visual.x - poseGPS.x; 
    arena_offset.y = pose_visual.y - poseGPS.y; 
		utm_offset.x = -poseUTM.x;
		utm_offset.y = -poseUTM.y;
  }
		poseUTM_corrected.x = poseUTM.x + utm_offset.x;
		poseUTM_corrected.y = poseUTM.y + utm_offset.y;
  poseArena = poseOdomToArena(poseOdom);
  tf_pub.sendTransform(tf::StampedTransform(tParentToChild, ros::Time::now(),
    arenaFrame, mapFrame));

		std::cout << " UTM("<<poseUTM_corrected.x << ","<<poseUTM_corrected.y<<")\n";
  std_msgs::String msg;
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(2) << 
    "Arena: ("<<poseArena.x<<","<<poseArena.y<<","<<poseArena.theta<<
    ") odom: ("<<poseOdom.x<<","<<poseOdom.y<<","<<poseOdom.theta<< ")"<<
    ") GPS: ("<<poseGPS.x<<","<<poseGPS.y<<","<<poseGPS.theta<< ")"<<
    ") offset: ("<<arena_offset.x<<","<<arena_offset.y<<","<<arena_offset.theta<< ")"<<
    ") avg : ("<<p_avg.x<<","<<p_avg.y<< ")"<<
    ") pose_visual : ("<<pose_visual.x<<","<<pose_visual.y<< ")"<< 
		" UTM("<<poseUTM_corrected.x << ","<<poseUTM_corrected.y<<")"<<
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
  odomGPSSubscriber = mNH.subscribe((name + "/odom/ekf"), 10, 
      &Localization::odomGPSHandler, this);
  odomUTMSubscriber = mNH.subscribe((name + "/odom/utm"), 10, 
      &Localization::odomUTMHandler, this);
  status_publisher = mNH.advertise<std_msgs::String>((name + "/loc_status"), 1, true);
  arenaFrame = name+"/arena";
  mapFrame = name+"/map";
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  tParentToChild.setRotation(q);

}
geometry_msgs::Pose2D Localization::getPoseUTM(void) 
{
	geometry_msgs::Pose2D pose;
	pose.x = -poseUTM_corrected.x;
	pose.y = -poseUTM_corrected.y;
  return pose;
}

geometry_msgs::Pose2D Localization::getPoseOdom(void) 
{
  return poseOdom;
}
geometry_msgs::Pose2D Localization::getPoseArena(void) 
{
  return poseArena;
}

// convert a pose in the arena frame to a pose in the odom frame
geometry_msgs::Pose2D Localization::poseOdomToArena( 
    geometry_msgs::Pose2D poseOdom2D
    )
{
  geometry_msgs::Pose2D p;
  
  p.x = poseGPS.x + arena_offset.x;
  p.y = poseGPS.y + arena_offset.y;
  p.theta = poseOdom2D.theta;
  return p;
}
// convert a pose in the arena frame to a pose in the odom frame
geometry_msgs::Pose2D Localization::poseArenaToOdom( 
    geometry_msgs::Pose2D poseArena2D
    )
{
  geometry_msgs::Pose2D p;
  p.x = poseArena2D.x - arena_offset.x - poseGPS.x + poseOdom.x;
  p.y = poseArena2D.y - arena_offset.y - poseGPS.y + poseOdom.y;
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
  //Get (x,y) location directly from pose
  poseOdom.x = message->pose.pose.position.x;
  poseOdom.y = message->pose.pose.position.y;

  //Get theta rotation by converting quaternion orientation to pitch/roll/yaw
  poseOdom.theta = thetaFromQuat(message->pose.pose.orientation);
  //TODO trying to find bottleneck	poseArena = poseOdomToArena(poseOdom);
}

void Localization::odomGPSHandler(const nav_msgs::Odometry::ConstPtr& message) {
  //Get (x,y) location directly from pose
  poseGPS.x = message->pose.pose.position.x;
  poseGPS.y = message->pose.pose.position.y;

  //Get theta rotation by converting quaternion orientation to pitch/roll/yaw
  poseGPS.theta = thetaFromQuat(message->pose.pose.orientation);
  //TODO trying to find bottleneck	poseArena = poseOdomToArena(poseOdom);
}
void Localization::odomUTMHandler(const nav_msgs::Odometry::ConstPtr& message) {
  //Get (x,y) location directly from pose
  poseUTM.x = message->pose.pose.position.x;
  poseUTM.y = message->pose.pose.position.y;
}
