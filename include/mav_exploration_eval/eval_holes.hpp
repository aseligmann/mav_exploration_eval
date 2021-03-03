#pragma once           // Only include once per compile
#ifndef EVAL_HOLES  // Conditional compiling
#define EVAL_HOLES

// Includes
#include <ros/ros.h>  // ROS header


// Define class
class EvalHoles {
public:
  // Constructor and destructor
  EvalHoles(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~EvalHoles();

  // Public functions
  

  // Public variables and objects

private:
  // Private functions


  // Private variables and objects
  ros::NodeHandle nh_;
};

#endif