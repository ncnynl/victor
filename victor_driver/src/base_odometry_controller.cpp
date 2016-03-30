#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>

class BaseOdometryDriveController
{
private:

  ros::NodeHandle _nh;

  ros::Publisher _cmd_vel_pub;

  tf::TransformListener _listener;

public:
  // ROS node initialization
  BaseOdometryDriveController(ros::NodeHandle &nh)
  {
    _nh = nh;
    
    //set up the publisher for the cmd_vel topic
    _cmd_vel_pub = _nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
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
    
    ros::Rate rate(10.0);
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
};

int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "odom_drive_controller");
  ros::NodeHandle nh;

  BaseOdometryDriveController driver(nh);
  driver.DriveForward(0.5);
}