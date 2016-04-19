#ifndef PID_POSITION_CONTROLLER_CORE_H
#define PID_POSITION_CONTROLLER_CORE_H

#include "position.h"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include <queue>

class PositionPID
{
public:

  PositionPID(ros::Publisher pub, double tol_dist, double tol_angle, double dist, double ang);

  ~PositionPID();

  // Publish Command Velocity Twist Messages from PID
  void publish_message(double angle_twist, double velocity_twist);

  // Callback for published odometry messages.
  void message_callback(const nav_msgs::Odometry::ConstPtr& msg);

  // Check to see if robot has arrived at final position.  With error tolerance
  bool has_arrived(Position &current);
  
  // Reset Controller to Initial Conditions
  void reset();
  
private:
  
  double calculate_PSD(double current_value, double last_value, double set_point, double dt, double k_p, double k_d, double k_s, double &sum);
  
  //variables
  Position start;     // Start position. Distance will be measured from here
  Position last;      // Last position of robot.
  Position current;   // Current position of robot
  
  double tolerance_distance;   // Tolerated deviation from target distance.
  double tolerance_angle; // Tolerated deviation from target angle.
  
  ros::Publisher pub_message; // Object for publishing messages.
  double target_distance; // Robot will go forward this distance.
  double target_angle;    // Robot will turn by this angle.
  int iterations;        // Number of received messages.
  double sum_distance;    // Sum of distance errors for PSD controller.
  double sum_angle;       // Sum of angle errors for PSD controller.

  double k_p_distance;
  double k_s_distance;
  double k_d_distance;
  
  double k_p_angle;
  double k_s_angle;
  double k_d_angle;
  
};

#endif