#include <Wire.h>
#include <EEPROM.h>
#include <I2Cdev.h>
#include <RTIMUSettings.h>
#include <RTIMUBNO055.h>
#include <CalLib.h>

RTIMU *imu;                                           // the IMU object
RTIMUSettings *settings;                              // the settings object

//  SERIAL_PORT_SPEED defines the speed to use for the serial port

#define  SERIAL_PORT_SPEED  57600

long lastSendMessageTime;
long lastReceiveMessageTime;

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Imu.h>
ros::NodeHandle  nh;
//std_msgs::Empty imuMsg;
//ros::Publisher imuPub("imu/data", &imuMsg);

sensor_msgs::Imu imuMsg2;
ros::Publisher imuPub2("imu/data2", &imuMsg2);

void setup()
{  
    int errcode;
    
   // Serial.begin(SERIAL_PORT_SPEED);
   ///// while (!Serial) {
        ; // wait for serial port to connect. 
    //}
    
    nh.initNode();
    nh.advertise(imuPub2);
    
    Wire.begin();   
 
    settings = new RTIMUSettings();
    imu = RTIMU::createIMU(settings);                        // create the imu object
  //  Serial.print("rosteensyrtimulib starting using device "); Serial.println(imu->IMUName());
    if ((errcode = imu->IMUInit()) < 0) {
      //Serial.print("Failed to init IMU: "); Serial.println(errcode);
    }
}

void loop()
{ 
   // RTIMU_DATA imuData;
 
    if (imu->IMURead()) {                                // get the latest data if ready yet
      //  imuData = imu->getIMUData();
       // imuMsg.header.stamp = nh.now();
        //imuMsg.header.frame_id = "imu_link";
       // imuMsg.orientation.x = imuData.fusionQPose.x();
       // imuMsg.orientation.y = imuData.fusionQPose.y();
      //  imuMsg.orientation.z = imuData.fusionQPose.z();
      //  imuMsg.orientation.w = imuData.fusionQPose.scalar();
      //  imuMsg.angular_velocity.x = imuData.gyro.x();
     //   imuMsg.angular_velocity.y = imuData.gyro.y();
      //  imuMsg.angular_velocity.z = imuData.gyro.z();
      //  imuMsg.linear_acceleration.x = imuData.accel.x() * 9.81;
      //  imuMsg.linear_acceleration.y = imuData.accel.y() * 9.81;
      //  imuMsg.linear_acceleration.z = imuData.accel.z() * 9.81;
imuMsg2.seq = 10;
        imuPub2.publish(&imuMsg2);
    }

    nh.spinOnce();
}
