#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <victor_driver/OdomNavGoalAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "umb_mark_calibration");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<victor_driver::OdomNavGoalAction> ac("odom_nav", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  victor_driver::OdomNavGoalGoal goal;
  goal.x = 0.0; // 1 meter forward
  goal.theta = M_PI / 2.0; // Turn 90 Degrees
  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(300.0)); // 5 Mins

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  
  goal.x = 0;
    goal.theta = M_PI / 2.0; // Turn 90 Degrees
  ac.sendGoal(goal);

  //wait for the action to return
  finished_before_timeout = ac.waitForResult(ros::Duration(300.0)); // 5 Mins

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");
  
  //exit
  return 0;
}