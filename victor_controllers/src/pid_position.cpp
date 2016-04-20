#include "pid_position.h"
#include <math.h>

#define F_KP 2.58  // P constant for PSD translation controller
#define F_KD 0.047  // D constant for PSD translation controller
#define F_KI 0.0  // S constant for PSD translation controller

#define R_KP 2.0  // P constant for PSD rotation controller
#define R_KD 0.1  // D constant for PSD rotation controller
#define R_KI 0.0  // S constant for PSD rotation controller

PositionPID::PositionPID(ros::Publisher pub, double tol_dist, double tol_angle, double dist, double ang)
{
  target_distance = dist;
  tolerance_distance = tol_dist;
  tolerance_angle = tol_angle;
  
  target_angle = ang;
  pub_message = pub;

  reset();
}

PositionPID::~PositionPID()
{
}

void PositionPID::reset()
{
  iterations = 0;
  sum_distance = 0;
  sum_angle = 0;
  
  start.x = start.y = start.theta = 0.0;
  last.x = last.y = last.theta = 0.0;
  current.x = current.y = current.theta = 0.0;
  
  start.time = last.time = current.time = ros::Time::now();
}

//Publisher
void PositionPID::publish_message(double angle_twist, double velocity_twist)
{
  //preparing message
  geometry_msgs::Twist msg;

  msg.linear.x = velocity_twist;
  msg.angular.z = angle_twist;

  //publishing message
  pub_message.publish(msg);
}

//Subscriber
void PositionPID::message_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  double angle_command = 0;
  double speed_command = 0;

  current.x = msg->pose.pose.position.x;
  current.y = msg->pose.pose.position.y;
  current.theta = 2.0*asin(msg->pose.pose.orientation.z);
  current.time = msg->header.stamp;
  
  if (has_arrived(current) == true)
  {
    ROS_INFO("GOAL ACHIEVED");
    publish_message(0.0,0.0);
    //exit(0);
  }
  if (iterations == 0)
  {
    start.x = current.x;
    start.y = current.y;
    start.time = current.time;
    start.theta = current.theta;
    last.x = current.x;
    last.y = current.y;
    last.time = current.time;
    last.theta = current.theta;
  }
  iterations++;

  // Calculate Sample Time Period
  double dt = current.time.toSec() - last.time.toSec();
  //Calculation of action intervention.
  if (fabs(target_distance) > tolerance_distance)
  {
    speed_command = calculate_PSD(start.get_distance(current)*copysign(1.0, target_distance),start.get_distance(last)*copysign(1.0, target_distance),target_distance,dt, F_KP,F_KD,F_KI, sum_distance);
  }

  if (current.theta-last.theta < -M_PI)
  {
    current.theta += 2*M_PI;
  } 
  else if (current.theta-last.theta > M_PI)
  {
    current.theta -= 2*M_PI;
  }

  angle_command = calculate_PSD(current.theta - start.theta, last.theta - start.theta, target_angle, target_angle, dt, R_KD, R_KI, sum_angle);

  //Saving position to last
  last.x = current.x;
  last.y = current.y;
  last.time = current.time;
  last.theta = current.theta;

  //Invoking method for publishing message
  publish_message(angle_command, speed_command);
}

bool PositionPID::has_arrived(Position &current)
{
  double distance;
  distance = start.get_distance(current)*copysign(1.0, target_distance);
  if (fabs(distance-target_distance) > tolerance_distance) // TODO: Rolling Mean error
  {
    return false;
  }
  if (fabs(target_angle - (current.theta - start.theta)) > tolerance_angle &
    fabs(target_angle - (current.theta - start.theta) + 2 * M_PI) > tolerance_angle &
    fabs(target_angle - (current.theta - start.theta) - 2 * M_PI) > tolerance_angle)
  {
    return false;
  }
  return true;
}

double PositionPID::calculate_PSD(double current_value, double last_value, double set_point, double dt, double k_p, double k_d, double k_s, double &sum)
{
  double speed = 0;
  double error = set_point - current_value;
  double previousError = set_point - last_value;
  double derivative = (error - previousError)/dt;
  sum = sum + error*dt;
  speed = k_p*error + k_d*derivative + k_s*(sum);
  return speed;
}