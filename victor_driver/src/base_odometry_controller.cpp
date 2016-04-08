#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <victor_driver/OdomDriveAction.h>
#include <victor_driver/OdomTurnAction.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>

using namespace boost::accumulators;
//typedef actionlib::SimpleActionServer<victor_driver::OdomNavGoalAction> Server;

class BaseOdometryDriveController
{
protected:

  ros::NodeHandle _nh;

  ros::Publisher _cmd_vel_pub;

  tf::TransformListener _listener;

  // Setpoint publisher
  ros::Publisher _set_point_dist_pub;
  ros::Publisher _set_point_heading_pub;
  
   // Advertise a plant state msg
  std_msgs::Float64 plant_state;
  ros::Publisher _state_dist_pub;
ros::Publisher _state_heading_pub;
  // States
  std_msgs::Float64 _dist_state;
  std_msgs::Float64 _heading_state;
  
  // Subscribe to "control_effort" topic to get a controller_msg.msg
  ros::Subscriber _control_effort_dist_sub;
  ros::Subscriber _control_effort_heading_sub;
  
  actionlib::SimpleActionServer<victor_driver::OdomDriveAction> _as_drive; 
  actionlib::SimpleActionServer<victor_driver::OdomTurnAction> _as_turn; 
  
  victor_driver::OdomDriveResult _drive_result;
  victor_driver::OdomTurnResult  _turn_result;
  
  // Control Efforts
  double control_effort_distance;
  double control_effort_heading;
  
  std::string _base_frame;
  std::string _odom_frame;

  double _base_odom_controller_rate;
  double _base_odom_controller_timeout;
  
  double _forward_speed;
  double _rotate_speed;
  
public:
  // ROS node initialization
  BaseOdometryDriveController(ros::NodeHandle &nh) :
  _as_drive(_nh, "odom_drive", boost::bind(&BaseOdometryDriveController::executeDriveCB, this, _1), false),
  _as_turn(_nh, "odom_turn", boost::bind(&BaseOdometryDriveController::executeTurnCB, this, _1), false),
  control_effort_distance(0),
  control_effort_heading(0)
  {
    _nh = nh;
  
    // Get Base Params
   _nh.param<std::string>("base_odometry_controller/base_frame", _base_frame, "base_link");
   _nh.param<std::string>("base_odometry_controller/odom_frame", _odom_frame, "odom");
   
   _nh.param("base_odometry_controller/base_odom_controller_rate", _base_odom_controller_rate, 50.0);
   _nh.param("base_odometry_controller/base_odom_controller_timeout", _base_odom_controller_timeout, 3.0);
   
   _nh.param("base_odometry_controller/forward_speed", _forward_speed, 0.25);
   _nh.param("base_odometry_controller/rotate_speed", _rotate_speed, 0.75);
   
    //set up the publisher for the cmd_vel topic
    _cmd_vel_pub = _nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
   
    ROS_INFO("Starting Odom Drive Action Server");
    _as_drive.start();

    ROS_INFO("Starting Odom Turn Action Server");
    _as_turn.start();

    ROS_INFO("Starting setpoint publisher");
    _set_point_dist_pub = _nh.advertise<std_msgs::Float64>("/odom_distance/setpoint", 1);
     _set_point_heading_pub = _nh.advertise<std_msgs::Float64>("/odom_heading/setpoint", 1);
    ROS_INFO("Starting state publishers");
    _state_dist_pub = _nh.advertise<std_msgs::Float64>("/odom_distance/state", 1);
    _state_heading_pub = _nh.advertise<std_msgs::Float64>("/odom_heading/state", 1);
   ROS_INFO("Starting control effort subscriber");
   
    _control_effort_dist_sub = _nh.subscribe("/odom_distance/control_effort", 1, &BaseOdometryDriveController::ControlEffortDistanceCallback, this);
    _control_effort_heading_sub = _nh.subscribe("/odom_heading/control_effort", 1, &BaseOdometryDriveController::ControlEffortHeadingCallback, this);
  }
  
  void ControlEffortDistanceCallback(const std_msgs::Float64& control_effort_input)
  {
   // ROS_INFO("CONTROL EFFORT: %f",  control_effort_input.data);
   control_effort_distance = control_effort_input.data;
  }
  
    void ControlEffortHeadingCallback(const std_msgs::Float64& control_effort_input)
  {
   // ROS_INFO("CONTROL EFFORT: %f",  control_effort_input.data);
   control_effort_heading = control_effort_input.data;
  }

  void executeDriveCB(const victor_driver::OdomDriveGoalConstPtr& goal)
  {
    ROS_INFO("EXECUTING DRIVE ACTION!");
    double distance_moved = 0;
    bool success = DriveForward(goal->target_distance, distance_moved);
    _drive_result.distance_moved = distance_moved;
    if(success)
    {
      _as_drive.setSucceeded(_drive_result);
    }
  }
  
  void executeTurnCB(const victor_driver::OdomTurnGoalConstPtr& goal)
  {
    ROS_INFO("EXECUTING TURN ACTION!");
    double angle_turned = 0;
    bool success = Turn(goal->target_angle, angle_turned);
    _turn_result.angle_turned = angle_turned;
    if(success)
    {
      _as_turn.setSucceeded(_turn_result);
    }
  }
  
  // Drive forward a specified distance based on odometry information
  bool DriveForward(double distance, double &distance_moved)
  {
    int forwardScale = 1;

    bool forward = (distance >= 0);
    
    if(!forward)
      forwardScale = -1;
    
    distance = fabs(distance);
   
    std_msgs::Float64 setpoint;
    setpoint.data = 0;
    _set_point_dist_pub.publish(setpoint);
  
        setpoint.data = distance;
    _set_point_dist_pub.publish(setpoint);
    //wait for the listener to get the first message 
    // TODO: Make the transform links parameter arguments
    _listener.waitForTransform(_base_frame, _odom_frame, 
                               ros::Time(0), ros::Duration(1.0));
    
    //we will record transforms here
    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;

    //record the starting transform from the odometry to the base frame
    _listener.lookupTransform(_base_frame, _odom_frame, 
                              ros::Time(0), start_transform);
    
    //we will be sending commands of type "twist"
    geometry_msgs::Twist base_cmd;
    
    //the command will be to go forward at 0.25 m/s
    base_cmd.linear.y = base_cmd.angular.z = 0;
    
    accumulator_set<double, stats<tag::rolling_mean> > error_acc(tag::rolling_window::window_size = 50);
    ros::Rate rate(_base_odom_controller_rate);
    bool done = false;
    while (!done && _nh.ok())
    {
      // Check Preempted
         if (_as_drive.isPreemptRequested())
         {
           ROS_INFO("Drive Request: Preempted");
           // set the action state to preempted
           _as_drive.setPreempted();
           done = false;
           break;
         }
         
      // Update Velocities
      base_cmd.linear.x = forwardScale * _forward_speed * control_effort_distance;
      
      //send the drive command
      _cmd_vel_pub.publish(base_cmd);

      //get the current transform
      try
      {
        _listener.lookupTransform(_base_frame, _odom_frame, 
                                  ros::Time(0), current_transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
        break;
      }
      //see how far we've traveled
      tf::Transform relative_transform = start_transform.inverse() * current_transform;
      distance_moved = relative_transform.getOrigin().length();

      ROS_INFO_THROTTLE(0.1, "Distance Moved: %f / %f",distance_moved, distance_moved - distance);
     // std::cout << "Distance Moved: " << distance_moved << " Error: " << fabs(distance_moved - distance) << std::endl;
      error_acc(distance_moved - distance);

      
      _dist_state.data = distance_moved;
      _state_dist_pub.publish(_dist_state);
      
      if(fabs(rolling_mean(error_acc)) < 0.01) 
      {
	ROS_INFO("Distance Final: %f / %f", distance_moved, rolling_mean(error_acc));
	base_cmd.linear.x = 0;
	//send the drive command to stop
	_cmd_vel_pub.publish(base_cmd);
	done = true;
      }
      rate.sleep();
    }
    
    if (done) 
    {
      ros::Duration(1.0).sleep();
      
      //get the current transform
      try
      {
        _listener.lookupTransform(_base_frame, _odom_frame, 
                                  ros::Time(0), current_transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
	  return false;
      }
      //see how far we've traveled
      tf::Transform relative_transform = start_transform.inverse() * current_transform;
      distance_moved = relative_transform.getOrigin().length();
      return true;
    }
    return false;
  }
  bool Turn(double radians, double &angle_turned)
  {
    
    bool clockwise = true;
    
    if(radians < 0)
      clockwise = false;
    
    radians = fabs(radians);
    
    // Map Zero to 2*PI ( 360 deg )
    while(radians < 0) radians += 2*M_PI;
    while(radians > 2*M_PI) radians -= 2*M_PI;

    std_msgs::Float64 setpoint;
    setpoint.data = 0;
    _set_point_heading_pub.publish(setpoint);

    setpoint.data = radians;
    _set_point_heading_pub.publish(setpoint);
    
    //wait for the listener to get the first message
    _listener.waitForTransform(_base_frame, _odom_frame, 
                               ros::Time(0), ros::Duration(1.0));
    
    //we will record transforms here
    tf::StampedTransform start_transform;
    tf::StampedTransform previous_transform;
    tf::StampedTransform current_transform;

    //record the starting transform from the odometry to the base frame
    _listener.lookupTransform(_base_frame, _odom_frame, 
                              ros::Time(0), start_transform);
    
    previous_transform = start_transform;
    //we will be sending commands of type "twist"
    geometry_msgs::Twist base_cmd;
    base_cmd.linear.x = base_cmd.linear.y = 0.0;
        base_cmd.angular.z = 0;
	
    float angular_z = _rotate_speed;

   if (clockwise) 
      angular_z = -_rotate_speed;
    
    //the axis we want to be rotating by
    tf::Vector3 desired_turn_axis(0,0,1);
    if (!clockwise) 
      desired_turn_axis = -desired_turn_axis;
    
    _heading_state.data = 0;
    _state_heading_pub.publish(_heading_state);
    
   accumulator_set<double, stats<tag::rolling_mean> > error_acc(tag::rolling_window::window_size = 20);
   
    ros::Rate rate(_base_odom_controller_rate);
    bool done = false;
    while (!done && _nh.ok())
    {
      
      // Check Preempted
      if (_as_turn.isPreemptRequested())
      {
	ROS_INFO("Turn Request: Preempted");
        // set the action state to preempted
        _as_turn.setPreempted();
        done = false;
	break;
      }
         
         // Update Velocities
      base_cmd.angular.z = angular_z * (control_effort_heading);
      //send the drive command
      _cmd_vel_pub.publish(base_cmd);

      //get the current transform
      try
      {
        _listener.lookupTransform(_base_frame, _odom_frame, 
                                  ros::Time(0), current_transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
        break;
      }
      tf::Transform relative_transform = 
        previous_transform.inverse() * current_transform;
	
	tf::Transform relative_transform_absolute = 
        start_transform.inverse() * current_transform;
	
      tf::Vector3 actual_turn_axis = 
        relative_transform.getRotation().getAxis();
    //  angle_turned += relative_transform.getRotation().getAngle();
      double angle_turned_absolute = relative_transform_absolute.getRotation().getAngle();
      previous_transform = current_transform;
      if ( fabs(angle_turned_absolute) < 1.0e-2) 
      {
	_heading_state.data = angle_turned_absolute;
	_state_heading_pub.publish(_heading_state);
      
	rate.sleep();
	continue;
      }

     // if ( actual_turn_axis.dot( desired_turn_axis ) < 0 ) 
     // {
	
	//ROS_INFO("TURN AXIS FLIP!!!");
        //angle_turned = 2 * M_PI - angle_turned;
	//angle_turned_absolute = 2 * M_PI - angle_turned_absolute;
      //}

     // ROS_INFO_THROTTLE(0.1, "Angle Turned ABS: %f",angle_turned_absolute);
      error_acc(angle_turned_absolute - radians);
      if(fabs(rolling_mean(error_acc)) < 0.005) 
      {
	ROS_INFO("Angle Turned: %f / %f", angle_turned_absolute, rolling_mean(error_acc));
	base_cmd.angular.z = 0;
	//send the drive command
        _cmd_vel_pub.publish(base_cmd);
	done = true;
      }
      
      _heading_state.data = angle_turned_absolute;
      _state_heading_pub.publish(_heading_state);
      
      rate.sleep();
    }
    if (done) 
    {
      ros::Duration(1.0).sleep();
      try
      {
        _listener.lookupTransform(_base_frame, _odom_frame, 
                                  ros::Time(0), current_transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
	return false;
      }
      
      
      	tf::Transform relative_transform_absolute = 
        start_transform.inverse() * current_transform;
	
      angle_turned = relative_transform_absolute.getRotation().getAngle();
      
      ROS_INFO("Angle Turned Final: %f", angle_turned);
      return true;
    }
    return false;
  }

};

int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "odom_drive_controller");
  ros::NodeHandle nh;

  BaseOdometryDriveController driver(nh);
  //driver.DriveForward(5.0);
  ros::spin();
  return 0;
  //driver.DriveForward(5.0);
  //ros::Duration(0.5).sleep();
  //driver.Turn(-2 * M_PI);
  //ros::Duration(0.5).sleep();
  //driver.DriveForward(5.0);
  
}