#ifndef BASE_CONTROLLER_CORE_H
#define BASE_CONTROLLER_CORE_H

// ROS includes.
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <victor_msgs/MotorEncoder.h>
#include <victor_msgs/MotorControl.h>
#include <std_srvs/Empty.h>

// Dynamic reconfigure
#include <victor_driver/BaseControllerConfig.h>

//#include "microcontroller_bridge.h"

using std::string;

class BaseController
{
public:
  //! Constructor.
  BaseController();

  //! Destructor.
  ~BaseController();

  bool init();
  void update();
  void spin();
  void stop();
  void reset();
private:
  
  std::string _base_frame;
  std::string _odom_frame;

  double _base_controller_rate;
  double _base_controller_timeout;

  bool _stopped;

  // Platform Options
  double _wheel_diameter;
  double _wheel_radius;
  double _wheel_track;
  double _encoder_resolution;
  double _gear_reduction;
  double _ticks_per_meter;
  
  // Calibration Data
  double _c_left;
  double _c_right;
  double _c_wheel_base;
  
  // Position Data -> To Odometry Class
  float _x;
  float _y;
  float _th;

  // Encoder Data
  int _encoder_left_prev;
  int _encoder_right_prev;
  int _encoder_left;
  int _encoder_right;
  
  // Speed Data (PPS aka ints)
  int _v_target_left;
  int _v_target_right;

  ros::Time _last_cmd_vel;

  ros::Time _current_time, _last_time;
  ros::Duration _dt;
 // MicroControllerBridge _microcontroller;

  // Node NodeHandle
  ros::NodeHandle _nh;
  
  // Publishers
  ros::Publisher _odom_pub;
  ros::Publisher _motor_speed_pub;
  ros::Publisher _reset_pub;
  tf::TransformBroadcaster odom_broadcaster;
  
  // Subscribers
  ros::Subscriber _cmd_vel_sub;
  ros::Subscriber _motor_enc_sub;

  // Services
  ros::ServiceClient _reset_driver_service;
  ros::ServiceServer _service;
  
  // Callbacks
  void motor_enc_callback(const victor_msgs::MotorEncoder& encoder_val);   	
  void cmd_vel_callback(const geometry_msgs::Twist& vel_cmd);
  bool reset_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
  /*********************
  ** Dynamic Reconfigure
  **********************/
  dynamic_reconfigure::Server<victor_driver::BaseControllerConfig> *dynamic_reconfigure_server;
  dynamic_reconfigure::Server<victor_driver::BaseControllerConfig>::CallbackType dynamic_reconfigure_callback;
  void reconfigure_callback(victor_driver::BaseControllerConfig &config, uint32_t level);
  
};

#endif // BASE_CONTROLLER_CORE_H
