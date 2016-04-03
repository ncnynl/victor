#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <actionlib/client/simple_action_client.h>
#include <victor_driver/OdomDriveAction.h>
#include <victor_driver/OdomTurnAction.h>

#include <string>

class UMBMark
{
protected:

  ros::NodeHandle _nh;

  ros::Subscriber _laser_scan_sub;

  // Action Clients
      actionlib::SimpleActionClient<victor_driver::OdomDriveAction> _ac_drive;
    actionlib::SimpleActionClient<victor_driver::OdomTurnAction> _ac_turn;
    
    // Measurement Distances Around Robot.
    double left_distance;
    double right_distance;
    double front_distance;
    double rear_distance;
    
    // Mean Errors
    double x_cg_cw;
    double y_cg_cw;
    
    double x_cg_ccw;
    double y_cg_ccw;
    
    // Calibration Calculation Values
    double alpha_x;
    double alpha_y;
    double beta_x;
    double beta_y;
    
    double R;
    double e_b; // Effective Wheelbase
    double e_d; // Effective Wheel Diameter
    double x_initial, x_final; // Initial and Final Wall Measurements
    double y_initial, y_final; // Initial and Final Wall Measurements
    
    double b_actual; // Actual Wheel Base
    
    double _calib_distance; // Square Leg Distance
    // Measurement Controls
    int max_measurements;
    int num_measurements;
    int num_runouts;
public:
  // ROS node initialization
  UMBMark(ros::NodeHandle &nh) :
  _ac_drive("odom_drive", true),
  _ac_turn("odom_turn", true)
  {
    _nh = nh;
    
    max_measurements = 5;
    num_measurements = 0;
    
    num_runouts = 2; // Average Number Runs
    left_distance = 0;
    right_distance = 0;
    front_distance = 0;
    rear_distance = 0;
    
    x_cg_cw = 0;
    y_cg_cw = 0;
    
    x_cg_ccw = 0;
    y_cg_ccw = 0;

     alpha_x = 0;
     alpha_y = 0;
     beta_x = 0;
     beta_y = 0;
    
     R = 0;
     e_b = 0; // Effective Wheelbase
     e_d = 0; // Effective Wheel Diameter
     x_initial = 0;
     x_final = 0; // Initial and Final Wall Measurements
     y_initial = 0;
     y_final = 0; // Initial and Final Wall Measurements
    
     b_actual = 0; // Actual Wheel Base
    
    // Subscribe to scan topic
    ROS_INFO("Subscribing to Scan Topic For Range Measurements");
    _laser_scan_sub = _nh.subscribe<sensor_msgs::LaserScan>("scan",max_measurements, &UMBMark::processScan, this);
   
  }
  
  void Calibrate(double calib_distance)
  {
    _calib_distance = calib_distance;
    victor_driver::OdomDriveGoal drive_goal;
    drive_goal.target_distance = _calib_distance; // 2 meter forward
	_ac_drive.sendGoal(drive_goal);
	
	_ac_drive.waitForResult(); // Wait Indefinitely.  May make a global timeout parameter
	actionlib::SimpleClientGoalState state = _ac_drive.getState();
	ROS_INFO("Drive finished: %s.  Moved: %f",state.toString().c_str(), _ac_drive.getResult()->distance_moved);
	return;
    std::cout << "Please Place Mobile Platform at Start Position (CW Run). Press ENTER when finishing." << std::endl;
    std::cin.get();
    std::cout << "Please Clear the Area.  Starting in 5 seconds." << std::endl;
    ros::Duration(5.0).sleep();
    CalibrateCW();
    
    std::cout << "Please Place Mobile Platform at Start Position (CCW Run). Press ENTER when finishing." << std::endl;
    std::cin.get();
    std::cout << "Please Clear the Area.  Starting in 5 seconds." << std::endl;
    ros::Duration(5.0).sleep();
    CalibrateCCW();
    
    // Run Calibration Calculations
    alpha_x = (x_cg_cw + x_cg_ccw) / (-4 * _calib_distance);
    alpha_y = (y_cg_cw - y_cg_ccw) / (-4 * _calib_distance);
    
    beta_x = (x_cg_cw - x_cg_ccw) / (-4 * _calib_distance);
    beta_y = (y_cg_cw + y_cg_ccw) / (-4 * _calib_distance);
    
    R = _calib_distance * 0.5 / sin(beta_x * 0.5);
    
    ROS_INFO("Alpha: X=%f, Y=%f", alpha_x, alpha_y);
    ROS_INFO("Beta: X=%f, Y=%f", beta_x, beta_y);
    ROS_INFO("R: R=%f", R);
    
    double wb = .458; // TODO: From Parameter
    e_d = (R + (wb * 0.5))/(R - (wb * 0.5));
    ROS_INFO("Effective Diameter, e_d: %f", e_d);
    
    e_b = (M_PI/2)/((M_PI/2)-alpha_x);
    ROS_INFO("Effective Base, e_b: %f", e_b);
    
    ROS_INFO("Actual Base, b_actual: %f", wb * e_b);
    

  }
  void CalibrateCW()
  {
    
    // send a goal to the action
    victor_driver::OdomDriveGoal drive_goal;
    victor_driver::OdomTurnGoal turn_goal;
   
   x_cg_cw = y_cg_cw = 0; // Zero out error
   
   for(int i = 0; i < num_runouts; i++)
   {
         
      // Get Initial Measurements (CW)
      resetRangeMeasurements();
      spin(); // Get Range Measurements
      processRangeMeasurements();
   
      x_initial = left_distance;
      y_initial = rear_distance;
	
      ROS_INFO("Initial Distances: %f, %f", x_initial, y_initial);
	// CW Run
      for (uint8_t j = 0; j < 4; j++) 
      {
	std::cout << "Driving forward " << _calib_distance << "m" << std::endl;
	drive_goal.target_distance = _calib_distance; // 2 meter forward
	_ac_drive.sendGoal(drive_goal);
	
	_ac_drive.waitForResult(); // Wait Indefinitely.  May make a global timeout parameter
	actionlib::SimpleClientGoalState state = _ac_drive.getState();
	ROS_INFO("Drive finished: %s.  Moved: %f",state.toString().c_str(), _ac_drive.getResult()->distance_moved);
	ros::Duration(2.0).sleep();

	std::cout << "Turning 90 Degrees CW" << std::endl;
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
      
      x_cg_cw += (x_final - x_initial);
      y_cg_cw += (y_final - y_initial);
      
      ROS_INFO("Final Distances: %f, %f", x_final, y_final);
   
      ROS_INFO("Local|(Mean) Error CW: X=%f(%f), Y=%f(%f)", x_cg_cw, x_cg_cw / (i+1), y_cg_cw, y_cg_cw / (i+1));
      
      std::cout << "Run " << i+1 << " of " << num_runouts << " completed." << std::endl;
      if(i+1 < num_runouts)
      {
	std::cout << "Reposition Mobile Platform (if necessary).  Starting in 10 seconds." << std::endl;
	ros::Duration(10.0).sleep();
      }
   }
   
   // Compute Mean Error
   x_cg_cw /= num_runouts;
   y_cg_cw /= num_runouts;
   
  }
  
  void CalibrateCCW()
  {
    
    // send a goal to the action
    victor_driver::OdomDriveGoal drive_goal;
    victor_driver::OdomTurnGoal turn_goal;
   
   
   x_cg_ccw = y_cg_ccw = 0; // Zero out error
   
   for(int i = 0; i < num_runouts; i++)
   {
         // Get Initial Measurements (CCW)
   resetRangeMeasurements();
   spin(); // Get Range Measurements
   processRangeMeasurements();
   
      x_initial = rear_distance;
      y_initial = right_distance;
	
      ROS_INFO("Initial Distances: %f, %f", x_initial, y_initial);
	// CCW Run
      for (uint8_t j = 0; j < 4; j++) 
      {
	std::cout << "Driving forward " << _calib_distance << "m" << std::endl;
	drive_goal.target_distance = _calib_distance; // 2 meter forward
	_ac_drive.sendGoal(drive_goal);
	
	_ac_drive.waitForResult(); // Wait Indefinitely.  May make a global timeout parameter
	actionlib::SimpleClientGoalState state = _ac_drive.getState();
	ROS_INFO("Drive finished: %s.  Moved: %f",state.toString().c_str(), _ac_drive.getResult()->distance_moved);
	ros::Duration(2.0).sleep();

	std::cout << "Turning 90 Degrees CCW" << std::endl;
	turn_goal.target_angle = -M_PI / 2.0; // 90 Degrees
	_ac_turn.sendGoal(turn_goal);
	_ac_turn.waitForResult(); // Wait Indefinitely.  May make a global timeout parameter
	state = _ac_turn.getState();
	
	ROS_INFO("Turn finished: %s.  Turned: %f",state.toString().c_str(), _ac_turn.getResult()->angle_turned);
	ros::Duration(2.0).sleep();
      }
      
      resetRangeMeasurements();
      spin(); // Get Range Measurements
      processRangeMeasurements();

      x_final = rear_distance;
      y_final = right_distance;

      
      x_cg_ccw += (x_final - x_initial);
      y_cg_ccw += (y_final - y_initial);
      
      ROS_INFO("Final Distances: %f, %f", x_final, y_final);
   
      ROS_INFO("Local|(Mean) Error CCW: X=%f(%f), Y=%f(%f)", x_cg_ccw, x_cg_ccw / (i+1), y_cg_ccw, y_cg_ccw / (i+1));
      
      std::cout << "Run " << i+1 << " of " << num_runouts << " completed." << std::endl;
      if(i+1 < num_runouts)
      {
	std::cout << "Reposition Mobile Platform (if necessary).  Starting in 10 seconds." << std::endl;
	ros::Duration(10.0).sleep();
      }
      
      
   }
   
   // Compute Mean Error
   x_cg_ccw /= num_runouts;
   y_cg_ccw /= num_runouts;
   
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


double getdouble()
{
  for(;;)
  {
    double value;
    std::cin >> value;
    if(std::cin.fail())  // cin failed?
    {
      std::cin.clear();  // reset cin
      std::string garbage;
      std::getline(std::cin, garbage); // ignore rest of line
      std::cout << "\nInvalid Input. Please try again.\n";
    }
    else
    {
      return value;
    }
  }
}


int main (int argc, char **argv)
{
  ros::init(argc, argv, "umb_mark_calibration");

  ros::NodeHandle nh;
  
  double calibDistance = 1.0; // Defaul 1 Meter
   UMBMark umb_mark(nh);
     
  std::cout << "Press any key to start UMB Mark Calibration." << std::endl;
  std::cin.get();

  
  std::cout << "Desired Calibration Square Side Length (m): " << std::endl;
  calibDistance = getdouble();
  std::cin.ignore();
  std::cout << "Running UMB Mark with "<< calibDistance << " m square" << std::endl;
  umb_mark.Calibrate(calibDistance);
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


