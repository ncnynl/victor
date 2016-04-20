// ---------------------------------------------------------------------------

// This example code was used to successfully communicate with 15 ultrasonic sensors. You can adjust

// the number of sensors in your project by changing SONAR_NUM and the number of NewPing objects in the

// "sonar" array. You also need to change the pins for each sensor for the NewPing objects. Each sensor

// is pinged at 33ms intervals. So, one cycle of all sensors takes 495ms (33 * 15 = 495ms). The results

// are sent to the "oneSensorCycle" function which currently just displays the distance data. Your project

// would normally process the sensor results in this function (for example, decide if a robot needs to

// turn and call the turn function). Keep in mind this example is event-driven. Your complete sketch needs

// to be written so there's no "delay" commands and the loop() cycles at faster than a 33ms rate. If other

// processes take longer than 33ms, you'll need to increase PING_INTERVAL so it doesn't get behind.

// ---------------------------------------------------------------------------

/* Ultrasound Helper Includes */
#include <NewPing.h>

/* IMU Includes */
#include <Wire.h>
#include <I2Cdev.h>
#include <RTIMUSettings.h>
#include <RTIMUBNO055.h>
#include <CalLib.h>
#include <EEPROM.h>

/* ROS Includes */
#include <ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>

#define SONAR_NUM     1 // Number of sensors.

#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.

#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

#define SERIAL_PORT_SPEED 115200

#define DISPLAY_INTERVAL 150 

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.

unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.

uint8_t currentSensor = 0;          // Keeps track of which sensor is active.



NewPing sonar[SONAR_NUM] = {     // Sensor object array.

  NewPing(12, 11, MAX_DISTANCE) // Each sensor's trigger pin, echo pin, and max distance to ping.

};


// IMU Configurations
#if !defined(BNO055_28) && !defined(BNO055_29)
#error "One of BNO055_28 or BNO055_29 must be selected in RTIMULibdefs.h"
#endif

RTIMUBNO055 *imu;                                     // the IMU object
RTIMUSettings settings;                               // the settings object

unsigned long lastDisplay;
unsigned long lastRate;
int sampleCount;

void setup() {

  Serial.begin(SERIAL_PORT_SPEED);

  pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.

  for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
  {
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
  }
  
  int errcode;
  
    Wire.begin();
    imu = (RTIMUBNO055 *)RTIMU::createIMU(&settings);                        // create the imu object
  
    Serial.print("ArduinoIMU starting using device "); Serial.println(imu->IMUName());
    if ((errcode = imu->IMUInit()) < 0) {
        Serial.print("Failed to init IMU: "); Serial.println(errcode);
    }
  
    lastDisplay = lastRate = millis();
    sampleCount = 0;


}



void loop() {

  unsigned long now = millis();
  unsigned long delta;
  int loopCount = 1;
    
  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.

    if (millis() >= pingTimer[i]) {         // Is it this sensor's time to ping?

      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.

      if (i == 0 && currentSensor == SONAR_NUM - 1) oneSensorCycle(); // Sensor ping cycle complete, do something with the results.

      sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).

      currentSensor = i;                          // Sensor being accessed.

      cm[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.

      sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).

    }

  }

  // Other code that *DOESN'T* analyze ping results can go here.
  while (imu->IMURead()) {                                // get the latest data if ready yet
        // this flushes remaining data in case we are falling behind
        if (++loopCount >= 10)
            continue;
        sampleCount++;
        if ((delta = now - lastRate) >= 1000) {
            Serial.print("Sample rate: "); Serial.println(sampleCount);
            sampleCount = 0;
            lastRate = now;
        }
        if ((now - lastDisplay) >= DISPLAY_INTERVAL) {
            lastDisplay = now;
//          RTMath::display("Gyro:", (RTVector3&)imu->getGyro());                // gyro data
//          RTMath::display("Accel:", (RTVector3&)imu->getAccel());              // accel data
//          RTMath::display("Mag:", (RTVector3&)imu->getCompass());              // compass data
            RTMath::displayRollPitchYaw("Pose:", (RTVector3&)imu->getFusionPose()); // fused output
            Serial.println();
        }
  }

}



void echoCheck() { // If ping received, set the sensor distance to array.

  if (sonar[currentSensor].check_timer())

    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;

}



void oneSensorCycle() { // Sensor ping cycle complete, do something with the results.

  // The following code would be replaced with your code that does something with the ping results.

  for (uint8_t i = 0; i < SONAR_NUM; i++) {

    //Serial.print(i);

    //Serial.print("=");

    //Serial.print(cm[i]);

    //Serial.print("cm ");

  }

  //Serial.println();

}
