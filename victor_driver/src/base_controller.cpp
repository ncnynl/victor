// ROS Includes
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
// Dynamic Reconfigure
#include <dynamic_reconfigure/server.h>


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
  _ticks_per_meter(1),
  _x(0),
  _y(0),
  _th(0),
  _encoder_left_prev(0),
  _encoder_right_prev(0),
  _encoder_left(0),
  _encoder_right(0),
  _v_target_left(0),
  _v_target_right(0),
  _last_cmd_vel(0),
  dynamic_reconfigure_server(NULL)
  {
    //_wheel_radius = _wheel_diameter * 0.5;
  }

BaseController::~BaseController()
{
    if (dynamic_reconfigure_server != NULL)
      delete dynamic_reconfigure_server;
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
   _nh.param<std::string>("base_controller/base_frame", _base_frame, "base_link");
   _nh.param<std::string>("base_controller/odom_frame", _odom_frame, "odom");
   _nh.param("base_controller/wheel_track", _wheel_track, 0.48);
   _nh.param("base_controller/wheel_diameter", _wheel_diameter, 0.2);
   _nh.param("base_controller/encoder_resolution", _encoder_resolution, 72000.0);
   _nh.param("base_controller/gear_reduction", _gear_reduction, 1.0);
   _nh.param("base_controller/base_controller_rate", _base_controller_rate, 10.0);
   _nh.param("base_controller/base_controller_timeout", _base_controller_timeout, 1.0);
  
   // Calibration Data
   _nh.param("base_controller/calibration_left_wheel", _c_left, 1.0);
   _nh.param("base_controller/calibration_right_wheel", _c_right, 1.0);
   _nh.param("base_controller/calibration_wheel_base", _c_wheel_base, 1.0);

  // Update Radius
  _wheel_radius = _wheel_diameter * 0.5;
  
  // Encoder Ticks per Meter of Travel
  _ticks_per_meter = _encoder_resolution * _gear_reduction / (_wheel_diameter * M_PI);

  // Time Variables
  _current_time = ros::Time::now();
  _last_time = ros::Time::now();
 
  _encoder_right_prev = _encoder_left_prev = 0;
  _encoder_right = _encoder_left = 0;

  // Update Publishers and Subscribers
  _odom_pub = _nh.advertise<nav_msgs::Odometry>(_odom_frame, 50);
  _motor_speed_pub = _nh.advertise<victor_msgs::MotorControl>("motor_speed", 20);
  _reset_pub = _nh.advertise<std_msgs::Empty>("controller_reset", 2);
  _cmd_vel_sub = _nh.subscribe("/cmd_vel", 1, &BaseController::cmd_vel_callback, this);

  _motor_enc_sub = _nh.subscribe("/motor_encoder", 5, &BaseController::motor_enc_callback, this);

  // Services
  ros::service::waitForService("reset_driver", 10000);
  _reset_driver_service = _nh.serviceClient<std_srvs::Empty>("reset_driver");

  // Reset Encoders
  reset();
  
  _service = _nh.advertiseService("reset_base", &BaseController::reset_callback, this);

  // Dynamic Reconfigure
  dynamic_reconfigure_callback = boost::bind(&BaseController::reconfigure_callback, this, _1, _2);

  dynamic_reconfigure_server = new dynamic_reconfigure::Server<victor_driver::BaseControllerConfig>();
  dynamic_reconfigure_server->setCallback(dynamic_reconfigure_callback);

}
void BaseController::reset()
{
  // Reset Driver
  std_srvs::Empty reset_msg;
  _reset_driver_service.call(reset_msg);
   
  // Reset Base
  _x = _y =_th = 0;
  _encoder_left_prev = _encoder_right_prev = _encoder_left =_encoder_right = 0;
  _v_target_left = _v_target_right = 0;

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
  
  // Update Time Values
  _dt = _current_time - _last_time;
  _last_time = _current_time;
  double dt = _dt.toSec();
  
  // Compute Odometry
  // TODO: Need Error Checking here if we didn't get encoder values
  d_right = (_encoder_right - _encoder_right_prev) / _ticks_per_meter;
  d_left = (_encoder_left - _encoder_left_prev) / _ticks_per_meter;
  // Cache the Previous Value
  _encoder_left_prev = _encoder_left;
  _encoder_right_prev = _encoder_right;
  
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
  //since all odometry is 6DOF we'll need a quaternion created from yaw
  //  geometry_msgs::Quaternion odom_quat;// = tf::createQuaternionMsgFromYaw(_th);
 // odom_quat.x = 0.0;
 // odom_quat.y = 0.0;
 // odom_quat.z = sin(_th / 2.0);
 // odom_quat.w = cos(_th / 2.0);
  
  
      // Compute and store orientation info
      const geometry_msgs::Quaternion orientation(
            tf::createQuaternionMsgFromYaw(_th));
      
  
  
    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = _current_time;
    odom_trans.header.frame_id = _odom_frame;
    odom_trans.child_frame_id = _base_frame;

    odom_trans.transform.translation.x = _x;
    odom_trans.transform.translation.y = _y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = orientation;

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
    odom.pose.pose.orientation = orientation;

    //covariance
    odom.pose.covariance[0]  = 0.01;
    odom.pose.covariance[7]  = 0.01;
    odom.pose.covariance[14] = 99999;
    odom.pose.covariance[21] = 99999;
    odom.pose.covariance[28] = 99999;
    odom.pose.covariance[35] = 0.01;

    
    //set the velocity
    odom.child_frame_id = _base_frame;
    odom.twist.twist.linear.x = v_xy;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = v_th;

    // Covariance
    
    //publish the message
    _odom_pub.publish(odom);
  
  if(!_stopped)
  {
    if(_current_time > _last_cmd_vel + ros::Duration(_base_controller_timeout))
    {
      _v_target_left = 0;
      _v_target_right = 0;
      stop();
    }
    else
    {
      victor_msgs::MotorControl motor_msg;
      motor_msg.left_speed = _v_target_left;
      motor_msg.right_speed = _v_target_right;

      //publish the message
      _motor_speed_pub.publish(motor_msg);
    }
  }
//ROS_INFO("Theta: %f\n", _th);
//std::cout << "Theta: " << _th << std::endl;
//ROS_INFO("Target: %d, %d", _v_target_left, _v_target_right);

}
void BaseController::stop()
{
  //ROS_INFO("Stopping!\n");
  // Set Motor Speeds to Zero/Off
  _stopped = true;
 // _microcontroller.drive(0,0);
  
    victor_msgs::MotorControl motor_msg;
    motor_msg.left_speed = 0;
    motor_msg.right_speed = 0;

    //publish the message
    _motor_speed_pub.publish(motor_msg);
}


void BaseController::cmd_vel_callback(const geometry_msgs::Twist& vel_cmd)
{
  //ROS_INFO("GETTING CALLBACK VELOCITY: %d\n", _stopped);
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

  _stopped = false;
}
void BaseController::motor_enc_callback(const victor_msgs::MotorEncoder& encoder_val)
{
   // ROS_INFO("GETTING CALLBACK: %d, %d", encoder_val.left_encoder, encoder_val.right_encoder);
   _encoder_left = encoder_val.left_encoder;
   _encoder_right = encoder_val.right_encoder;
}
void BaseController::reconfigure_callback(victor_driver::BaseControllerConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %f %f", 
            config.wheel_diameter, config.wheel_track);
  
  _wheel_diameter = config.wheel_diameter;
  _wheel_track = config.wheel_track;
  
    // Update Radius
  _wheel_radius = _wheel_diameter * 0.5;
  
  // Encoder Ticks per Meter of Travel
  _ticks_per_meter = _encoder_resolution * _gear_reduction / (_wheel_diameter * M_PI);

}
bool BaseController::reset_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  ROS_INFO("Resetting Base Controller");
  reset();
  return true;
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

