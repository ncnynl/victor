#include "position.h"

Position::Position(double x_pos, double y_pos, double theta, ros::Time t)
{
  x = x_pos;
  y = y_pos;
  theta = theta;
  time = t;
}

Position::Position()
{
  x = 0;
  y = 0;
  theta = 0;
  time = ros::Time::now();
}
Position::~Position()
{
}

double Position::get_angle(Position &target)
{
  return atan2((target.y - y),(target.x - x));
}

double Position::get_distance(Position &target)
{
  return sqrt((pow(target.y - y,2.0)) + (pow(target.x - x,2.0)));
}