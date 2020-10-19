#pragma once

#include <Eigen/Dense>
#include <ros/ros.h>


template<typename T>
bool getPublicParam(const std::string& name, T& parameter,
              const ros::NodeHandle& nh = ros::NodeHandle())
{
  if (nh.getParam(name, parameter)) {
    return true;
  }
  ROS_ERROR_STREAM("Could not load parameter" << name);
  return false;
};
