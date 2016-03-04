

/* Roboclaw includes */
#include <BMSerial.h>
#include <RoboClaw.h>

/* ROS Includes */
#include <ros.h>
#include <std_msgs/String.h>

#include <std_srvs/Empty.h>



#include <victor_msgs/MotorControl.h>
#include <victor_msgs/MotorEncoder.h>
#include <victor_msgs/MotorStatus.h>

#define address 0x80

#define ROSBAUDRATE 57600
#define ROBOCLAWBAUDRATE 38400

#define Kp 1.0
#define Ki 0.8
#define Kd 0.4
#define qpps 170000

#define AUTO_STOP_INTERVAL 2000
#define SERIAL_TIMEOUT 20 // MS
#define ENCODER_UPDATE_RATE 50 // 20 hz
#define STATUS_UPDATE_RATE 200 // Every 2 Seconds

using std_srvs::Empty;

// Setup communications with Robo Claw.  Hardware UART pines 19 and 18 (Receive/Transmit)
RoboClaw roboclaw(19,18, SERIAL_TIMEOUT * 1000); // Not sure why that is in microseconds.  Convert to ms

uint8_t status;
bool valid;

/* Encoder Counts */
int32_t enc1;
int32_t enc2;

/* Motor Timeout */
unsigned long last_motor_command = AUTO_STOP_INTERVAL;
/* Counters to track update rates for PID and Odometry */
unsigned long nextStatusUpdate = 0;
unsigned long nextEncoderUpdate = 0;
unsigned long nextRangeUpdate = 0;

long motor_left_speed_target;
long motor_right_speed_target;

long motor_left_speed_actual;
long motor_right_speed_actual;
unsigned char moving = 0; // is the base in motion?


// ROS Node Handle
ros::NodeHandle  nh;

// Services
void reset_encoders(const Empty::Request & req, Empty::Response & res)
{
  nh.loginfo("Encoder Reset");
  roboclaw.ResetEncoders(address);
}

ros::ServiceServer<Empty::Request, Empty::Response> reset_encoder_service("reset_encoders",&reset_encoders);

// Publishing
//std_msgs::String str_msg;
//ros::Publisher arduino_log("arduino_log", &str_msg);

victor_msgs::MotorEncoder encoder_msg;
ros::Publisher encoder_pub("motor_encoder", &encoder_msg);

victor_msgs::MotorStatus status_msg;
ros::Publisher status_pub("system_status", &status_msg);

// Subscribing
void motorControlCb( const victor_msgs::MotorControl& motorControl){

  
  /* Reset the auto stop timer */
    last_motor_command = millis();
    if (motorControl.left_speed == 0 && motorControl.right_speed == 0) 
    {
      moving = 0;
    }
    else 
    {
      moving = 1;
    }
    
    motor_left_speed_target = motorControl.left_speed;
    motor_right_speed_target = motorControl.right_speed;
    
    roboclaw.SpeedM1(address,motor_right_speed_target);
    roboclaw.SpeedM2(address,motor_left_speed_target);
    
    last_motor_command = millis();

//sprintf(logBuffer, "Got Motor Command: Left = %l Right = %l", leftTicks, rightTicks);
   //String logString = "Got Motor Command: Left = ";
  // logString += leftTicks;
  // logString += " Right = ";
  // logString += rightTicks;
  //str_msg.data = logString.c_str();
 // arduino_log.publish( &str_msg );
}

// Subscribe to relevant topics
ros::Subscriber<victor_msgs::MotorControl> motor_speed_subscriber("motor_speed", &motorControlCb );

void setup()
{
  // Init Motor Controller
  roboclaw.begin(ROBOCLAWBAUDRATE);
  
  // Stop Motors
  motor_stop();
  
  // Set MOtor PID Values.  
  // TODO: Pass in PID values from ROS Parameters
  roboclaw.SetM1VelocityPID(address,Kd,Kp,Ki,qpps);
  roboclaw.SetM2VelocityPID(address,Kd,Kp,Ki,qpps);
  
  // Setup ROS Serial 
  nh.getHardware()->setBaud(ROSBAUDRATE);
  nh.initNode();
  
  // Subscribe
  nh.subscribe(motor_speed_subscriber); // Motor Speed Subscriber
  
  // Advertise
  nh.advertise(encoder_pub); // Motor Encoder Values
  nh.advertise(status_pub);  // Motor Status
  nh.advertiseService(reset_encoder_service);
  
  //String logString = "Arduino Firmware Setup and Ready!";
  //str_msg.data = logString.c_str();
  nh.loginfo( "Arduino Firmware Setup and Ready!" );
}

void loop()
{
  
  // Check to see if we have exceeded the auto-stop interval
  if ((millis() - last_motor_command) > AUTO_STOP_INTERVAL) {
    motor_stop();
  }
  
  if (millis() > nextEncoderUpdate)
  {
    enc1 = roboclaw.ReadEncM1(address, &status, &valid);
    enc2 = roboclaw.ReadEncM2(address, &status, &valid);
    
    motor_left_speed_actual = roboclaw.ReadSpeedM2(address, &status, &valid);
    motor_right_speed_actual = roboclaw.ReadSpeedM1(address, &status, &valid);
    
    // Publish Encoder Here
    victor_msgs::MotorEncoder encoder_msg;
    encoder_msg.left_encoder = enc2;
    encoder_msg.right_encoder = enc1;
    encoder_msg.left_motor_speed = motor_left_speed_actual;
    encoder_msg.right_motor_speed = motor_right_speed_actual;
    encoder_pub.publish( &encoder_msg );
    nextEncoderUpdate = millis() + ENCODER_UPDATE_RATE;
  }

  if (millis() > nextStatusUpdate)
  {
    // Read Status
    victor_msgs::MotorStatus status_msg;
    
    long input = roboclaw.ReadMainBatteryVoltage(address);
    status_msg.battery_main_voltage = input / 10.0;
    
    input = roboclaw.ReadLogicBatteryVoltage(address);
    status_msg.battery_logic_voltage = input / 10.0;
    
    int16_t m1_current = 0;
    int16_t m2_current = 0;
    
    bool valid = roboclaw.ReadCurrents(address, m1_current, m2_current);
    status_msg.M1_current = m1_current / 100.0; // To AMPS
    status_msg.M2_current = m2_current / 100.0; // To AMPS
    
   uint16_t temp = 0;
   roboclaw.ReadTemp(address, temp);
   status_msg.board_temp = temp / 10.0;

    status_pub.publish( &status_msg );
    nextStatusUpdate = millis() + STATUS_UPDATE_RATE;
    
  }
  nh.spinOnce();
}

void motor_stop()
{
  // Stop Motors
  motor_left_speed_target = 0;
  motor_right_speed_target = 0;
  moving = 0;
  
  /* Causes Continual Current Draw Apparently */
  //roboclaw.SpeedM1(address, 0);
  //roboclaw.SpeedM2(address, 0);
  
  /* Use Non PID Version */
  roboclaw.ForwardM1(address, 0);
  roboclaw.ForwardM2(address, 0);
}

