/*    Copyright (C) 2021 Alessandro Saviolo
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

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
