#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include <string>

ros::Publisher pub;

void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  nav_msgs::Odometry o = *msg;
  std::string s(o.header.frame_id);
  std::string name = s.substr(0, s.find_last_of('/'));
  o.header.frame_id = name + "/map";
  pub.publish(o);
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, 0.0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, 
    ros::Time::now(), name + "/map", name + "/odom"));

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::NodeHandle rnh("~");
  std::string name;
  rnh.param<std::string>("name", name, "swarmie");
  pub = n.advertise<nav_msgs::Odometry>(name + "/odom/ekf", 1);
  ros::Subscriber sub = n.subscribe(name + "/odom/filtered", 
    1000, chatterCallback);

  ros::spin();
  return 0;
}
