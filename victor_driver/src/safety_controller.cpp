
// C++ Includes
#include <math.h>

// Node Includes
#include "safety_controller.h"
  
SafetyController::SafetyController() : 
  _safety_controller_rate(10.0), // 10 hz
  _safety_controller_timeout(1.0), // 1 Second
  bumper_front_triggered_(false),
  bumper_rear_triggered_(false),
  left_distance_(0),
  right_distance_(0),
  front_distance_(0),
  rear_distance_(0),
  max_measurements_(5),
  num_measurements_(0)
  {
  }

SafetyController::~SafetyController()
{
}
void SafetyController::spin()
{
    ros::Rate rate(_safety_controller_rate);
    while(ros::ok())
    {
        ros::spinOnce();
	update();
        rate.sleep();
    }
}
bool SafetyController::init()
{

  // Get Parameters from server

   _nh.param("safety_controller/front_bumper_distance", front_bumper_distance_, 0.48);
   _nh.param("safety_controller/rear_bumper_distance", rear_bumper_distance_, 0.2);
   _nh.param("safety_controller/safety_controller_rate", _safety_controller_rate, 10.0);
   _nh.param("safety_controller/safety_controller_timeout", _safety_controller_timeout, 1.0);
  

    // Update Publishers and Subscribers
    _laser_scan_sub = _nh.subscribe<sensor_msgs::LaserScan>("scan",max_measurements_, &SafetyController::process_scan, this);
    _cmd_vel_pub = _nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
}

void SafetyController::update()
{

   process_range_measurements();
   ROS_INFO("Got Scan Filtered: Rear - %f, Right - %f, Front - %f, Left - %f", rear_distance_, right_distance_,front_distance_,left_distance_);
   
}
void SafetyController::process_scan(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  // 0 -> Behind
  // 90 -> Right
  // 180 -> Front
  // 270 -> Left
  // Zero is Behind Robot (Technically Forward in neato_laser frame)
  // TODO: Get Transform for laser rotation.  (Make only work with planar laser)
  //ROS_INFO("Got Scan: %f, %f", scan->angle_min, scan->angle_max);
  ROS_INFO("Got Scan: 0 deg - %f, 90 deg - %f, 180 deg - %f, 270 deg - %f", scan->ranges[0], scan->ranges[90],scan->ranges[180],scan->ranges[270]);
  rear_distance_ += scan->ranges[0];
  right_distance_ += scan->ranges[90];
  front_distance_ += scan->ranges[180];
  left_distance_ += scan->ranges[270];
  num_measurements_++;
}

void SafetyController::process_range_measurements()
{
  left_distance_ = left_distance_ / num_measurements_;
  right_distance_ = right_distance_ / num_measurements_;
  front_distance_ = front_distance_ / num_measurements_;
  rear_distance_ = rear_distance_ / num_measurements_;    
}

 // Main Startup Function
int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "safety_controller");
  
  
  // Create Base Controller;
  SafetyController safety_controller;
  safety_controller.init();
  safety_controller.spin();
}

