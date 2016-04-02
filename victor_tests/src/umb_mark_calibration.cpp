#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <actionlib/client/simple_action_client.h>
#include <victor_driver/OdomDriveAction.h>
#include <victor_driver/OdomTurnAction.h>



class UMBMark
{
protected:

  ros::NodeHandle _nh;

  ros::Subscriber _laser_scan_sub;

  // Action Clients
      actionlib::SimpleActionClient<victor_driver::OdomDriveAction> _ac_drive;
    actionlib::SimpleActionClient<victor_driver::OdomTurnAction> _ac_turn;
    
    double left_distance;
    double right_distance;
    double front_distance;
    double rear_distance;
    
    double x_initial, x_final;
    double y_initial, y_final;
    int max_measurements;
    int num_measurements;
    
public:
  // ROS node initialization
  UMBMark(ros::NodeHandle &nh) :
  _ac_drive("odom_drive", true),
  _ac_turn("odom_turn", true)
  {
    _nh = nh;
    
    max_measurements = 5;
    num_measurements = 0;
    // Subscribe to scan topic
    ROS_INFO("Subscribing to Scan Topic For Range Measurements");
    _laser_scan_sub = _nh.subscribe<sensor_msgs::LaserScan>("scan",max_measurements, &UMBMark::processScan, this);
   /* 
    // TODO: Timeout for below and bail error
    ROS_INFO("Waiting for drive action server to start.");
    // wait for the action server to start
    _ac_drive.waitForServer(); //will wait for infinite time

    ROS_INFO("Waiting for turn action server to start.");
    // wait for the action server to start
    _ac_turn.waitForServer(); //will wait for infinite time
    
    ROS_INFO("Action servers started!");*/
  
   
  }
  
  void Calibrate()
  {

    // send a goal to the action
    victor_driver::OdomDriveGoal drive_goal;
    victor_driver::OdomTurnGoal turn_goal;

    // Get Initial Measurements (CW)
   resetRangeMeasurements();
   spin(); // Get Range Measurements
   processRangeMeasurements();
   
   x_initial = left_distance;
   y_initial = rear_distance;
    
   ROS_INFO("Initial Distances: %f, %f", x_initial, y_initial);
    // CW Run
   for (uint8_t i = 0; i < 4; i++) 
   {
     std::cout << "Driving forward 2.0m" << std::endl;
     drive_goal.target_distance = 2.0; // 2 meter forward
     _ac_drive.sendGoal(drive_goal);
     
     _ac_drive.waitForResult(); // Wait Indefinitely.  May make a global timeout parameter
     actionlib::SimpleClientGoalState state = _ac_drive.getState();
     ROS_INFO("Drive finished: %s.  Moved: %f",state.toString().c_str(), _ac_drive.getResult()->distance_moved);
     ros::Duration(2.0).sleep();

     std::cout << "Turning 90 Degrees" << std::endl;
     turn_goal.target_angle = M_PI / 2.0; // 90 Degrees
     _ac_turn.sendGoal(turn_goal);
     _ac_turn.waitForResult(); // Wait Indefinitely.  May make a global timeout parameter
     state = _ac_turn.getState();
     
     ROS_INFO("Turn finished: %s.  Turned: %f",state.toString().c_str(), _ac_turn.getResult()->angle_turned);
     ros::Duration(2.0).sleep();
   }
   
   resetRangeMeasurements();
   spin(); // Get Range Measurements
   processRangeMeasurements();
   
   x_final = left_distance;
   y_final = rear_distance;
   ROS_INFO("Final Distances: %f, %f", x_final, y_final);
   
   ROS_INFO("Error: %f, %f", x_final - x_initial, y_final - y_initial);
     
   
  }
  void spin()
  {
   ros::spinOnce(); 
  }
  void processScan(const sensor_msgs::LaserScan::ConstPtr& scan)
  {
    // 0 -> Behind
    // 90 -> Right
    // 180 -> Front
    // 270 -> Left
    // Zero is Behind Robot (Technically Forward in neato_laser frame)
    // TODO: Get Transform for laser rotation.  (Make only work with planar laser)
    //ROS_INFO("Got Scan: %f, %f", scan->angle_min, scan->angle_max);
    ROS_INFO("Got Scan: 0 deg - %f, 90 deg - %f, 180 deg - %f, 270 deg - %f", scan->ranges[0], scan->ranges[90],scan->ranges[180],scan->ranges[270]);
    rear_distance += scan->ranges[0];
    right_distance += scan->ranges[90];
    front_distance += scan->ranges[180];
    left_distance += scan->ranges[270];
    num_measurements++;
    
  }
  void processRangeMeasurements()
  {
    left_distance = left_distance / num_measurements;
    right_distance = right_distance / num_measurements;
    front_distance = front_distance / num_measurements;
    rear_distance = rear_distance / num_measurements;
    
  }
  void resetRangeMeasurements()
  {
    num_measurements = 0;
    left_distance = right_distance = front_distance = rear_distance = 0.0;
  }

};






int main (int argc, char **argv)
{
  ros::init(argc, argv, "umb_mark_calibration");

  ros::NodeHandle nh;
  
   UMBMark umb_mark(nh);
     
  std::cout << "Press any key to start UMB Mark Calibration." << std::endl;
  std::cin.get();
  
  umb_mark.Calibrate();
  //ros::spin();
  
/*
  //wait for the action to return
  bool finished_before_timeout = ac_drive.waitForResult(ros::Duration(300.0)); // 5 Mins

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac_drive.getState();
    //victor_driver::OdomDriveResult result = ac_drive.getResult();
    ROS_INFO("Action finished: %s.  Moved: %f",state.toString().c_str(), ac_drive.getResult()->distance_moved);
  }
  else
    ROS_INFO("Action did not finish before the time out."); */
  
  //exit
  return 0;
}


