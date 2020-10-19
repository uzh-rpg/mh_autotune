#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>


void odomCallback(
    const nav_msgs::Odometry& msg) {
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(
    msg.pose.pose.position.x,
    msg.pose.pose.position.y,
    msg.pose.pose.position.z));
  transform.setRotation(tf::Quaternion(
    msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.y,
    msg.pose.pose.orientation.z,
    msg.pose.pose.orientation.w));
  br.sendTransform(tf::StampedTransform(
    transform,
    ros::Time::now(),
    msg.header.frame_id,
    msg.child_frame_id));
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "odometry_broadcaster");
  ros::NodeHandle nh;
  ros::Subscriber sub = 
    nh.subscribe("/hummingbird/ground_truth/odometry", 1, &odomCallback);
  ros::spin();
  return 0;
};
