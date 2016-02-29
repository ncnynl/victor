#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
//#include "base_controller.h"

//BaseController *base_controller;

int main(int argc, char** argv){
  
  ros::init(argc, argv, "victor");
  ros::NodeHandle n;
  
  // Create Base Controller;
  //base_controller = new BaseController();
  //base_controller->init();
  
  // Subscribe to cmd_vel
  //ros::Subscriber sub = n.subscribe("/cmd_vel", 1, &BaseController::cmd_vel_callback, base_controller);
   
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(100.0); // Update Rate
  while(n.ok())
  {
    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    last_time = current_time;
    r.sleep();
  }
}

