#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <victor_driver/OdomNavGoalAction.h>


//typedef actionlib::SimpleActionServer<victor_driver::OdomNavGoalAction> Server;

class BaseOdometryDriveController
{
protected:

  ros::NodeHandle _nh;

  ros::Publisher _cmd_vel_pub;

  tf::TransformListener _listener;

  actionlib::SimpleActionServer<victor_driver::OdomNavGoalAction> _as; 
  
  //Server as_;
public:
  // ROS node initialization
  BaseOdometryDriveController(ros::NodeHandle &nh) :
  _as(_nh, "odom_nav", boost::bind(&BaseOdometryDriveController::executeCB, this, _1), false)
  {
    _nh = nh;
    
    //set up the publisher for the cmd_vel topic
    _cmd_vel_pub = _nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
   
    ROS_INFO("Starting Odom Action Server");
    _as.start();
    // Start the Server
  }

  void executeCB(const victor_driver::OdomNavGoalGoalConstPtr& goal)
  {
    ROS_INFO("EXECUTING ACTION!");
    
    DriveForward(goal->x);
    ros::Duration(1.0).sleep();
    Turn(goal->theta);
    
    _as.setSucceeded();
    
    // Do lots of awesome groundbreaking robot stuff here
   // as->setSucceeded();
  }
  
  // Drive forward a specified distance based on odometry information
  bool DriveForward(double distance)
  {
    
    bool forward = (distance >= 0);
    
    //wait for the listener to get the first message 
    // TODO: Make the transform links parameter arguments
    _listener.waitForTransform("base_link", "odom", 
                               ros::Time(0), ros::Duration(1.0));
    
    //we will record transforms here
    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;

    //record the starting transform from the odometry to the base frame
    _listener.lookupTransform("base_link", "odom", 
                              ros::Time(0), start_transform);
    
    //we will be sending commands of type "twist"
    geometry_msgs::Twist base_cmd;
    
    //the command will be to go forward at 0.25 m/s
    base_cmd.linear.y = base_cmd.angular.z = 0;
    base_cmd.linear.x = 0.25; // TODO: From Velocity Controller and cmd_vel_mux
    
    ros::Rate rate(50.0);
    bool done = false;
    while (!done && _nh.ok())
    {
      //send the drive command
      _cmd_vel_pub.publish(base_cmd);
      rate.sleep();
      //get the current transform
      try
      {
        _listener.lookupTransform("base_link", "odom", 
                                  ros::Time(0), current_transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
        break;
      }
      //see how far we've traveled
      tf::Transform relative_transform = start_transform.inverse() * current_transform;
      double distance_moved = relative_transform.getOrigin().length();

      ROS_INFO("Distance Moved: %f", distance_moved);
      if(distance_moved > distance) 
	done = true;
    }
    if (done) 
      return true;
    return false;
  }
  bool Turn(double radians)
  {
    
    bool clockwise = true;
    
    if(radians < 0)
      clockwise = false;
    
    radians = fabs(radians);
    
    // Map Zero to 2*PI ( 360 deg )
   // while(radians < 0) radians += 2*M_PI;
    //while(radians > 2*M_PI) radians -= 2*M_PI;

    //wait for the listener to get the first message
    _listener.waitForTransform("base_link", "odom", 
                               ros::Time(0), ros::Duration(1.0));
    
    //we will record transforms here
    tf::StampedTransform start_transform;
    tf::StampedTransform previous_transform;
    tf::StampedTransform current_transform;

    //record the starting transform from the odometry to the base frame
    _listener.lookupTransform("base_link", "odom", 
                              ros::Time(0), start_transform);
    
    previous_transform = start_transform;
    //we will be sending commands of type "twist"
    geometry_msgs::Twist base_cmd;
    //the command will be to turn at 0.75 rad/s
    base_cmd.linear.x = base_cmd.linear.y = 0.0;
    base_cmd.angular.z = 0.5;
    if (clockwise) 
      base_cmd.angular.z = -base_cmd.angular.z;
    
    //the axis we want to be rotating by
    tf::Vector3 desired_turn_axis(0,0,1);
    if (!clockwise) 
      desired_turn_axis = -desired_turn_axis;
    
    ros::Rate rate(50.0);
    double angle_turned = 0.0;
    bool done = false;
    while (!done && _nh.ok())
    {
      //send the drive command
      _cmd_vel_pub.publish(base_cmd);
      rate.sleep();
      //get the current transform
      try
      {
        _listener.lookupTransform("base_link", "odom", 
                                  ros::Time(0), current_transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
        break;
      }
      tf::Transform relative_transform = 
        previous_transform.inverse() * current_transform;
	
      tf::Vector3 actual_turn_axis = 
        relative_transform.getRotation().getAxis();
      angle_turned += relative_transform.getRotation().getAngle();
      
      previous_transform = current_transform;
      if ( fabs(angle_turned) < 1.0e-2) 
	continue;

      if ( actual_turn_axis.dot( desired_turn_axis ) < 0 ) 
        angle_turned = 2 * M_PI - angle_turned;

      ROS_INFO("Angle Turned: %f", angle_turned);
      if (angle_turned > radians) 
	done = true;
    }
    if (done) 
      return true;
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