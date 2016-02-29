// ROS Includes
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

// C++ Includes
#include <math.h>

// Node Includes
#include "base_controller.h"
  
BaseController::BaseController() : 
  _base_frame("base_link"), // For TF
  _odom_frame("odom"), // For TF
  _base_controller_rate(10.0), // 10 hz
  _base_controller_timeout(1.0), // 1 Second
  _stopped(false),
  _wheel_diameter(0.2), // 8 in
  _wheel_track(0.48),	  // ~19 in
  _encoder_resolution(72000), 
  _gear_reduction(1),
  _accel_limit(0.1),
_accel_rate(15000.0),
_decel_rate(15000.0),
  _ticks_per_meter(1),
  _max_accel(1.0),
  _x(0),
  _y(0),
  _th(0),
  _encoder_left(0),
  _encoder_right(0),
  _v_left(0),
  _v_right(0),
  _v_target_left(0),
  _v_target_right(0),
  _last_cmd_vel(0),
  _bad_encoder_count(0)
  {
    //_wheel_radius = _wheel_diameter * 0.5;
  }

BaseController::~BaseController()
{
}
void BaseController::spin()
{
  _current_time = ros::Time::now();
  _last_time = _current_time;
    ros::Rate rate(_base_controller_rate);
    while(ros::ok())
    {
        ros::spinOnce();
	_last_time = _current_time;
	_current_time = ros::Time::now();
	update();
        rate.sleep();
    }
}
bool BaseController::init()
{
  // Get Parameters from server
  std::string port;
  int baud;
  int timeout;
  
   _nh.param<std::string>("base_controller/port", port, "/dev/ttyS0");
   _nh.param<std::string>("base_controller/base_frame", _base_frame, "base_link");
   _nh.param<std::string>("base_controller/odom_frame", _odom_frame, "odom");
   _nh.param("base_controller/baud", baud, 38400);
   _nh.param("base_controller/timeout", timeout, 500);
   _nh.param("base_controller/wheel_track", _wheel_track, 0.48);
   _nh.param("base_controller/wheel_diameter", _wheel_diameter, 0.2);
   _nh.param("base_controller/encoder_resolution", _encoder_resolution, 72000.0);
   _nh.param("base_controller/gear_reduction", _gear_reduction, 1.0);
   _nh.param("base_controller/decel_rate", _decel_rate, 10000.0);
   _nh.param("base_controller/accel_rate", _accel_rate, 10000.0);
   _nh.param("base_controller/base_controller_rate", _base_controller_rate, 10.0);
   _nh.param("base_controller/base_controller_timeout", _base_controller_timeout, 1.0);

  // Update Radius
  _wheel_radius = _wheel_diameter * 0.5;
  //ROS_INFO("RADIUS: %f, %f", _wheel_radius, _wheel_diameter);
  // Encoder Ticks per Meter of Travel
  _ticks_per_meter = _encoder_resolution * _gear_reduction / (_wheel_diameter * M_PI);

  // Max Acceleration
  _max_accel = _accel_limit * _ticks_per_meter / _base_controller_rate;
  // Time Variables
  _current_time = ros::Time::now();
  _last_time = ros::Time::now();
  
  // Microcontroller Initializations/Connect
  _microcontroller.init(port, (unsigned long)baud, timeout);
  _microcontroller.connect();
  
  // Reset Encoders back to zero
  _microcontroller.reset_encoders();
  _encoder_right = _encoder_left = 0;
  
  // Update Publishers and Subscribers
  _odom_pub = _nh.advertise<nav_msgs::Odometry>(_odom_frame, 50);
  _cmd_vel_sub = _nh.subscribe("/cmd_vel", 1, &BaseController::cmd_vel_callback, this);
}
void BaseController::update()
{
  _current_time = ros::Time::now();
  
  // Variables
  float dxy_ave;
  float d_th;
  float v_xy;
  float v_th;
  float d_right;
  float d_left;
  
  // Get Encoder Values
  int left_enc;
  int right_enc;
  bool bSuccess = _microcontroller.get_encoder_ticks(right_enc, left_enc);
  if(!bSuccess)
  {
    ++_bad_encoder_count;
    ROS_ERROR("Bad Encoder Read Count: %d",  _bad_encoder_count);
    return;
  }

  
  // Update Time Values
  _dt = _current_time - _last_time;
  _last_time = _current_time;
  double dt = _dt.toSec();
  
  // Compute Odometry
  // TODO: Need Error Checking here if we didn't get encoder values
  d_right = (right_enc - _encoder_right) / _ticks_per_meter;
  d_left = (left_enc - _encoder_left) / _ticks_per_meter;
  // Cache the Value
  _encoder_left = left_enc;
  _encoder_right = right_enc;
  
  dxy_ave = (d_right + d_left) * 0.5;
  d_th = (d_right - d_left) / _wheel_track;
  v_xy = dxy_ave / dt;
  v_th = d_th / dt;
  
  float _x_prev = _x;
  if(dxy_ave != 0)
  {
    float d_x = cos(d_th) * dxy_ave;
    float d_y = -sin(d_th) * dxy_ave;
    _x += (cos(_th) * d_x - sin(_th) * d_y);
    _y += (sin(_th) * d_x + cos(_th) * d_y);
  }
  
  float vel = (_x - _x_prev) / dt;
  if(d_th != 0)
  {
    _th += d_th;
  }
//  ROS_INFO("Publish: %d\n", _encoder_left);
  //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat;// = tf::createQuaternionMsgFromYaw(_th);
  odom_quat.x = 0.0;
  odom_quat.y = 0.0;
  odom_quat.z = sin(_th / 2.0);
  odom_quat.w = cos(_th / 2.0);
  
  
    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = _current_time;
    odom_trans.header.frame_id = _odom_frame;
    odom_trans.child_frame_id = _base_frame;

    odom_trans.transform.translation.x = _x;
    odom_trans.transform.translation.y = _y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = _current_time;
    odom.header.frame_id = _odom_frame;

    //set the position
    odom.pose.pose.position.x = _x;
    odom.pose.pose.position.y = _y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = _base_frame;
    odom.twist.twist.linear.x = v_xy;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = v_th;

    //publish the message
    _odom_pub.publish(odom);
    
//   ROS_INFO("V TARGET: %d", _v_target_left); 
  if(_current_time > _last_cmd_vel + ros::Duration(_base_controller_timeout))
  {
    _v_target_left = 0;
    _v_target_right = 0;
  }
  
  // Left
  if(_v_left < _v_target_left)
  {
    _v_left += _accel_rate;
    if(_v_left > _v_target_left)
    {
      _v_left = _v_target_left;
    }
  }
  else
  {
    // v_left = min(_v_left + _max_accel, _v_target_left);
    _v_left -= _decel_rate;
    if(_v_left < _v_target_left)
    {
      _v_left = _v_target_left;
    }
  }
  
  // Right
    if(_v_right < _v_target_right)
  {
    _v_right += _accel_rate;
    if(_v_right > _v_target_right)
    {
      _v_right = _v_target_right;
    }

  }
  else
  {
    // _v_right = min(_v_right + _max_accel, _v_target_left);
    _v_right -= _decel_rate;
    if(_v_right < _v_target_right)
    {
      _v_right = _v_target_right;
    }
  }
  
  if(!_stopped)
  {
    _microcontroller.drive(_v_left,_v_right);
  }
//ROS_INFO("Actual: %d, %d", _v_left, _v_right);
//ROS_INFO("Target: %d, %d", _v_target_left, _v_target_right);

}
void BaseController::stop()
{
  // Set Motor Speeds to Zero/Off
  _stopped = true;
  _microcontroller.drive(0,0);
  
}


void BaseController::cmd_vel_callback(const geometry_msgs::Twist& vel_cmd)
{
  _last_cmd_vel = ros::Time::now();
  float x = vel_cmd.linear.x;
  float th = vel_cmd.angular.z;
  float right = 0;
  float left = 0;
//   if(x == 0) // Turning in place (This is wrong)
//   {
//     right = th * _wheel_track * _gear_reduction * 0.5;
//     left = -right;
//   }
//   else if(th == 0) // Pure Forward/Backwards Motion
//   {
//     left = right = x;
//   }
//   else// Turn Rotate. Reevaluate this
//   {
//     left = x - th * _wheel_track * _gear_reduction * 0.5;
//     right = x + th * _wheel_track * _gear_reduction * 0.5;
//   }
//   
  //left = (x - th * _wheel_track / 2.0) / _wheel_radius / (M_PI * 2); 
  //right = (x + th * _wheel_track / 2.0) / _wheel_radius / (M_PI * 2);
  left = (x - th * _wheel_track / 2.0); 
  right = (x + th * _wheel_track / 2.0);
  
  _v_target_left = (int)(left * _ticks_per_meter);
  _v_target_right = (int)(right * _ticks_per_meter);

}
 
 // Main Startup Function
int main(int argc, char** argv)
 {
  
  ros::init(argc, argv, "base_controller");
  // Create Base Controller;
  BaseController base_controller;
  base_controller.init();
  base_controller.spin();
}
