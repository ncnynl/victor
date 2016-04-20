#include "position.h"
#include "pid_position.h"

#define SUBSCRIBER_BUFFER_SIZE 1  // Size of buffer for subscriber.
#define PUBLISHER_BUFFER_SIZE 1000  // Size of buffer for publisher.
#define TOLERANCE 0.01  // Distance from the target distance, at which the distance will be considered as achieved.
#define TOLERANCE_ANGLE 0.02  // Differenc from target angle, which will be tolerated.
#define PUBLISHER_TOPIC "/cmd_vel"
#define SUBSCRIBER_TOPIC "odom"
#define PI 3.141592

int main(int argc, char **argv)
{
  //Initialization of node
  ros::init(argc, argv, "dead_reckon");
  ros::NodeHandle n;
/*
  if(argc < 3) {
        printf("You must provide two arguments\n Usage\n -f [link](#x) move forward x meters\n -r [link](#x) turn around by x radians\n");
        exit(0);
    }
    float distance = 0.0;
    float angle = 0.0;
    if (strcmp(argv[1],"-f") == 0)
    {
    if (1 == sscanf(argv[2], "%f", &distance))
    {
      ROS_INFO("Going forward %f meters.", distance);
    }
    else
    {
      ROS_INFO("Can not parse second argument.\nYou must provide numeric argument\n");
      ROS_INFO("Usage\n -f [link](#x) move forward x meters\n -r [link](#x) turn around by x radians\n");
      exit(0);
    }
  }
  else if(strcmp(argv[1],"-r") == 0)
  {
    if (1 == sscanf(argv[2]), "%f", &angle))
    {
      //Normalizing angle to interval <-pi-xi;pi+xi>
      while (fabs(angle) > PI+0.01)
      {
        if (angle > 0)
        {
          angle -= 2*PI;
        }else
        {
          angle += 2*PI;
        }
      }
      ROS_INFO("Turning by %f radians.", angle);
    }
    else
    {
      ROS_INFO("Can not parse second argument.\nYou must provide numeric argument\n");
      ROS_INFO("Usage\n -f [link](#x) move forward x meters\n -r [link](#x) turn around by x radians\n");
      exit(0);
    }
  }
  else
  {
    ROS_INFO("Usage\n -f [link](#x) move forward x meters\n -r [link](#x) turn around by x radians");
    exit(0);
  }*/

  //Creating publisher
  ros::Publisher pubMessage = n.advertise<geometry_msgs::Twist>(PUBLISHER_TOPIC, PUBLISHER_BUFFER_SIZE);

  //Creating object, which stores data from sensors and has methods for
  //publishing and subscribing
  PositionPID *pose_PID = new PositionPID(pubMessage, TOLERANCE, TOLERANCE_ANGLE, 5, 0);

  //Creating subscriber and publisher
  ros::Subscriber sub = n.subscribe(SUBSCRIBER_TOPIC, SUBSCRIBER_BUFFER_SIZE, &PositionPID::message_callback, pose_PID);
  ros::spin();

  return 0;
}