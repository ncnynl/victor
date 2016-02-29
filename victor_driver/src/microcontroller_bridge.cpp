// Standard Includes
#include <string>

// ROS Includes
#include <ros/ros.h>

// Node Includes
#include "microcontroller_bridge.h"


MicroControllerBridge::MicroControllerBridge() : 
  _port("/dev/ttyS0"), 
  _baud(57600),
  _timeout(500) // 0.5 seconds or 500 milliseconds
  {

  }

MicroControllerBridge::~MicroControllerBridge()
{
  if(_serial != NULL)
  {
    _serial->close();
  }
}

bool MicroControllerBridge::init(const std::string &port, unsigned long baud, int timeout)
{
  _port = port;
  _baud = baud;
  _timeout = timeout;
  
  // Don't open yet.  But set it up.
  _serial = new serial::Serial();
  _serial->setPort(_port);
  _serial->setBaudrate(_baud);
  
  // Set Timeout
  serial::Timeout timeout_ = serial::Timeout::simpleTimeout(_timeout);
  _serial->setTimeout(timeout_);
}
void MicroControllerBridge::connect()
{
  ROS_INFO("[MicroControllerBridge::connect]: Connecting to Microcontroller on Serial port: %s...", _port.c_str());
  if(_serial->isOpen())
  {
    ROS_WARN("[MicroControllerBridge::connect]: Serial port: %s already open", _port.c_str());
    return;
  }
  
   try
   {
     // Open the port
     _serial->open();
   
     if(!_serial->isOpen())
     {
       ROS_ERROR("[MicroControllerBridge::connect]: Error opening Serial port: %s", _port.c_str());
      throw serial::SerialException("Unable to Open Port!");
       
    }
ros::Duration(1.0).sleep();
     unsigned long testBaud = get_baud();

    // ROS_INFO("LOOKING FOR: %d/%d", _baud, testBaud);
     if(testBaud != _baud)
     {
       // Sleep And Try Again
      ros::Duration(2.0).sleep();
      testBaud = get_baud();
      if(testBaud != _baud)
      {
	throw serial::SerialException("Improper Baud Rate Established!");
      }
     }
     // Give Firmware a second to wake up
     ros::Duration(1.0).sleep();
     ROS_INFO("[MicroControllerBridge::connect]: Connected Successfully to Microcontroller on Serial port: %s...", _port.c_str());
      ROS_INFO("[MicroControllerBridge::connect]: Microcontroller Bridge Ready");
     
  }
   catch(serial::SerialException &ex)
   {
     ROS_ERROR("[MicroControllerBridge::connect]: Serial Exception:");
     ROS_ERROR("%s",ex.what());
     ROS_ERROR("[MicroControllerBridge::connect]: Unable to connect to Microcontroller!");
     exit(-1);
   }
   catch(serial::IOException &ex)
   {
     ROS_ERROR("[MicroControllerBridge::connect]: I/O Exception:");
     ROS_ERROR("%s",ex.what());
     ROS_ERROR("[MicroControllerBridge::connect]: Unable to connect to Microcontroller!");
     exit(-1);
   }
}
void MicroControllerBridge::close()
{
  _serial->close();
}

// Serial Execution Functions
int MicroControllerBridge::execute(const string& szCommand)
{
  int ntries = 1;
  int attempts = 0;
  std::string szValue;
  try
  {
     do
     {
      _serial->flushInput();
      _serial->write(szCommand);
      _serial->write("\r\n");
      szValue = _serial->readline();
      attempts++;
     }
    while(attempts < ntries && szValue.empty() || szValue == "Invalid Command");
  }
  catch(...)
  {
    ROS_ERROR("[MicroControllerBridge::execute]: General Exception:");
    return -1; // TODO: Really should be a bool based on success/
  }
  return atoi(szValue.c_str());
}

bool MicroControllerBridge::execute_ack(const std::string &szCommand)
{
  int ntries = 1;
  int attempts = 0;
  std::string szAck;
  try
  {
     do
     {
      _serial->flushInput();
      _serial->write(szCommand);
      _serial->write("\r\n");
      szAck = _serial->readline();
      attempts++;
     }
    while(attempts < ntries && szAck.empty() || szAck == "Invalid Command");
  }
  catch(...)
  {
    ROS_ERROR("[MicroControllerBridge::execute_ack]: General Exception:");
    return false;
  }
  
  if(szAck == "OK")
    return true;
  else
    return false;
  
}
std::vector<int> MicroControllerBridge::execute_array(const std::string &szCommand)
{
  int ntries = 1;
  int attempts = 0;
  std::string szValue;
  std::vector<std::string> szValues;
  std::vector<int> nValues;
  try
  {
     do
     {
     //  ROS_INFO("Value String:");
      _serial->flushInput();
      _serial->write(szCommand);
      _serial->write("\r\n");
      szValue = _serial->readline();
      //ROS_INFO("Value String: %s", szValue.c_str());
      attempts++;
     }
    while(attempts < ntries && szValue.empty() || szValue == "Invalid Command");
  }
  catch(...)
  {
    ROS_ERROR("[MicroControllerBridge::execute_array]: General Exception:");
    return nValues;
  }
  
  // Convert to int
  split(szValue, ' ', szValues);
  convertToInt(nValues, szValues);
  return nValues;
        
}
// Query Functions
int MicroControllerBridge::get_baud()
{
  return execute("b");
}
bool MicroControllerBridge::get_encoder_ticks(int& left_enc, int& right_enc)
{
  std::vector<int> nValues = execute_array("e");
  if(nValues.size() != 2)
  {
    return false;
  }
  
  left_enc = nValues[0];
  right_enc = nValues[1];
  
  return true;
}

// Encoder Functions
bool MicroControllerBridge::reset_encoders()
{
  return execute_ack("r");
}

// Drive Functions
bool MicroControllerBridge::drive(int left, int right)
{
  char buffer[50];
  sprintf (buffer, "m %d %d", left, right);
  return execute_ack(buffer);
}


bool MicroControllerBridge::convertToInt(std::vector<int>& values, 
                              const std::vector<std::string>& array)
{
    values.resize(array.size());
    for(int i=0;i!=array.size();i++)
    {
      errno = 0;
      char *end;
      long lnum = strtol(array[i].c_str(), &end, 10);        //10 specifies base-10
      if (end == array[i].c_str())     //if no characters were converted these pointers are equal
      {
	ROS_ERROR("ERROR: can't convert string to number: %s", array[i].c_str());
	return false;
      }
      values[i]=(int)lnum;
    }
    return true;
}

void MicroControllerBridge::split(const std::string& s, char c, std::vector<string>& v) {
   string::size_type i = 0;
   string::size_type j = s.find(c);

   while (j != string::npos) {
      v.push_back(s.substr(i, j-i));
      i = ++j;
      j = s.find(c, j);

      if (j == string::npos)
         v.push_back(s.substr(i, s.length()));
   }
}

