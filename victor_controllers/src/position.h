#ifndef POSITION_CORE_H
#define POSITION_CORE_H

#include <math.h> 
#include "ros/ros.h"

class Position
{
public:

  Position();
  Position(double x_pos, double y_pos, double theta, ros::Time t);

  ~Position();


  // Angle Between 2 Points
  double get_angle(Position &target);

  // Distance Between 2 Points
  double get_distance(Position &target);

  double x;       // X Position
  double y;       // Y Position
  ros::Time time; // Measurement Time
  double theta;   // Orientation Angle
  
};
#endif