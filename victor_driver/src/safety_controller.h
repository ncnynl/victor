#ifndef SAFETY_CONTROLLER_CORE_H
#define SAFETY_CONTROLLER_CORE_H

// ROS includes.
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

// Dynamic reconfigure
//#include <victor_driver/BaseControllerConfig.h>

//#include "microcontroller_bridge.h"

class SafetyController
{
public:
  //! Constructor.
  SafetyController();

  //! Destructor.
  ~SafetyController();

  bool init();
  void update();
  void spin();

private:

  double _safety_controller_rate;
  double _safety_controller_timeout;

  // Measurement Distances Around Robot.
    double left_distance_;
    double right_distance_;
    double front_distance_;
    double rear_distance_;
    double max_measurements_;
    double num_measurements_;
    
    
    double front_bumper_distance_;
    double rear_bumper_distance_;
    bool bumper_front_triggered_;
    bool bumper_rear_triggered_;
  
  // Node NodeHandle
  ros::NodeHandle _nh;
  
  // Publishers
  ros::Publisher _cmd_vel_pub;
  
  // Subscribers
  ros::Subscriber _laser_scan_sub;
  
  // Services
  
  // Callbacks
  void process_scan(const sensor_msgs::LaserScan::ConstPtr& scan);
  void process_range_measurements();
};

#endif // SAFETY_CONTROLLER_CORE_H
