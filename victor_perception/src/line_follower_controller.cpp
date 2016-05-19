#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <geometry_msgs/Twist.h>

using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";
static const std::string OPENCV_WINDOW2 = "Image window2";

class LineFollowerController
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
  ros::Publisher _cmd_vel_pub;
public:
  LineFollowerController()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/rgb/image_color", 1, &LineFollowerController::imageCb, this, image_transport::TransportHints("compressed"));

    //_cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1);
    _cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    
    //image_pub_ = it_.advertise("/color_tracker/output_image_raw", 1);

  //  cv::namedWindow(OPENCV_WINDOW);
   //// cv::namedWindow(OPENCV_WINDOW2);
  }

  ~LineFollowerController()
  {
    //cv::destroyWindow(OPENCV_WINDOW);
    //cv::destroyWindow(OPENCV_WINDOW2);
  }
  

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv_bridge::CvImage HSV;
    cv_bridge::CvImage Mask;
    cv_bridge::CvImage Masked;
    cv_bridge::CvImage ControlImage;
    
    // Convert to HSV
    cv::cvtColor(cv_ptr->image, HSV.image, cv::COLOR_BGR2HSV);
    
     // Filter HSV store filtered in threshold matrix
  //cv::inRange(HSV.image, cv::Scalar(0, 50, 100), cv::Scalar(38, 255, 200), Mask.image);
   cv::inRange(HSV.image, cv::Scalar(17, 8, 147), cv::Scalar(38, 162, 256), Mask.image);
  
    // Apply Mask
    //cv::bitwise_and(cv_ptr->image, cv_ptr->image, Masked.image, Mask.image);
    int img_height = cv_ptr->image.rows;
    int img_width = cv_ptr->image.cols;
    
    int search_top = 0.9 * img_height;
    int search_bot = search_top + 20;
    
    Mask.image(Range(0, search_top), Range(0, img_width)) = Scalar::all(0);
    Mask.image(Range(search_bot, img_height), Range(0, img_width)) = Scalar::all(0);
    
    // TODO: Find Countours.  Remove Below Size
    Moments M = moments(Mask.image);
    if(M.m00 > 50) // Have Area
    {
      int cx = (int) (M.m10 / M.m00);
      int cy = (int) (M.m01 / M.m00);
      cv::circle(cv_ptr->image, cv::Point(cx, cy), 20, CV_RGB(0,0,255), -1);

      float err = cx - img_width * 0.5;
      
      geometry_msgs::Twist base_cmd;
      base_cmd.linear.x = 0.8;
      base_cmd.angular.z = -err / 100.0;
      
      //send the drive command
      _cmd_vel_pub.publish(base_cmd);
    }
    else
    {
      geometry_msgs::Twist base_cmd;
      base_cmd.linear.x = 0.0;
      base_cmd.angular.z = 0.0;
      
      //send the drive command
      _cmd_vel_pub.publish(base_cmd);
    }
    
    // Update GUI Window
   // cv::imshow(OPENCV_WINDOW2, cv_ptr->image);
   // cv::imshow(OPENCV_WINDOW, Mask.image);
    cv::waitKey(3);


    // Output modified video stream
    //HSV.encoding = sensor_msgs::image_encodings::BGR8;
    //image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "line_follower_controller");
  LineFollowerController ic;
  ros::spin();
  return 0;
}

