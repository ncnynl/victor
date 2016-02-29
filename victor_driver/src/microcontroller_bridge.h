#ifndef MICROCONTROLLER_BRIDGE_CORE_H
#define MICROCONTROLLER_BRIDGE_CORE_H

// ROS includes.
#include <ros/ros.h>

// Library Includes
#include "serial/serial.h"

using std::string;

class MicroControllerBridge
{
public:
  //! Constructor.
  MicroControllerBridge();

  //! Destructor.
  ~MicroControllerBridge();

  bool init(const std::string &szPort, unsigned long baud, int timeout);
  void update();
  void connect();
  void close();
  
  // Motor Controls
  bool drive(int left, int right);
  bool stop();
  
  // Encoder Controls
  bool reset_encoders();
  bool get_encoder_ticks(int &left_enc, int &right_enc);
  
  // Additional Functions
  int get_baud();
private:
  int execute(const std::string &szCommand);
  bool execute_ack(const std::string &szCommand);
  std::vector<int>  execute_array(const std::string &szCommand);
  
  void split(const std::string& s, char c, std::vector<string>& v);
  bool convertToInt(std::vector<int>& values, const std::vector<std::string>& array);
  unsigned long _baud;
  int _timeout;
  std::string _port;
  
  serial::Serial *_serial;
};

#endif // MICROCONTROLLER_BRIDGE_CORE_H
