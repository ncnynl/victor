#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";

int H_MIN = 8;
int H_MAX = 54;
int S_MIN = 0;
int S_MAX = 256;
int V_MIN = 229;
int V_MAX = 256;

const int FRAME_WIDTH = 1280;
const int FRAME_HEIGHT = 720;

const int MAX_NUM_OBJECTS = 50;
const int MIN_OBJECT_AREA = 10 * 10;
const int MAX_OBJECT_AREA = FRAME_HEIGHT * FRAME_WIDTH / 1.5;

int x = 0, y = 0; // Tracked Object Location

bool useMorphOps = true;
bool trackObjects = true;
class ColorTrackerController
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  ColorTrackerController()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/color_tracker/input_image_raw", 1,     
      &ColorTrackerController::imageCb, this);

    image_pub_ = it_.advertise("/color_tracker/output_image_raw", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ColorTrackerController()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }
  
  void morphOps(cv::Mat &thresh){

    //create structuring element that will be used to "dilate" and "erode" image.

    //the element chosen here is a 3px by 3px rectangle
    cv::Mat erodeElement = cv::getStructuringElement( MORPH_RECT,Size(3,3));
    
    //dilate with larger element so make sure object is nicely visible
    cv::Mat dilateElement = cv::getStructuringElement( MORPH_RECT,Size(8,8));

    cv::erode(thresh,thresh,erodeElement);
    cv::erode(thresh,thresh,erodeElement);
            
    cv::dilate(thresh,thresh,dilateElement);
    cv::dilate(thresh,thresh,dilateElement);
  }
  void trackFilteredObject(int &x, int &y, Mat threshold, Mat &cameraFeed){

	Mat temp;
	threshold.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );
	//use moments method to find our filtered object
	double refArea = 0;
	bool objectFound = false;
	if (hierarchy.size() > 0) {
		int numObjects = hierarchy.size();
        //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
        if(numObjects<MAX_NUM_OBJECTS){
			for (int index = 0; index >= 0; index = hierarchy[index][0]) {

				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;

				//if the area is less than 20 px by 20px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
                if(area>MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea){
					x = moment.m10/area;
					y = moment.m01/area;
					objectFound = true;
					refArea = area;
				}else objectFound = false;


			}
			//let user know you found an object
			if(objectFound ==true){
				putText(cameraFeed,"Tracking Object",Point(0,50),2,1,Scalar(0,255,0),2);
				//draw object location on screen
				drawObject(x,y,cameraFeed);}

		}else putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
	}
}
void drawObject(int x, int y,Mat &frame){

	//use some of the openCV drawing functions to draw crosshairs
	//on your tracked image!

    //UPDATE:JUNE 18TH, 2013
    //added 'if' and 'else' statements to prevent
    //memory errors from writing off the screen (ie. (-25,-25) is not within the window!)

	circle(frame,Point(x,y),20,Scalar(0,255,0),2);
    if(y-25>0)
    line(frame,Point(x,y),Point(x,y-25),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(x,0),Scalar(0,255,0),2);
    if(y+25<FRAME_HEIGHT)
    line(frame,Point(x,y),Point(x,y+25),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(x,FRAME_HEIGHT),Scalar(0,255,0),2);
    if(x-25>0)
    line(frame,Point(x,y),Point(x-25,y),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(0,y),Scalar(0,255,0),2);
    if(x+25<FRAME_WIDTH)
    line(frame,Point(x,y),Point(x+25,y),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(FRAME_WIDTH,y),Scalar(0,255,0),2);

	putText(frame,intToString(x)+","+intToString(y),Point(x,y+30),1,1,Scalar(0,255,0),2);

}
string intToString(int number){


	std::stringstream ss;
	ss << number;
	return ss.str();
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
    cv_bridge::CvImage Threshold;
    
    // Convert to HSV
    cv::cvtColor(cv_ptr->image, HSV.image, cv::COLOR_BGR2HSV);
    
    // Filter HSV store filtered in threshold matrix
    cv::inRange(HSV.image, cv::Scalar(H_MIN, S_MIN, V_MIN), cv::Scalar(H_MAX, S_MAX, V_MAX), Threshold.image);
    
    if(useMorphOps)
      morphOps(Threshold.image);
    
    if(trackObjects)
      trackFilteredObject(x,y,Threshold.image,cv_ptr->image);
    
    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, Threshold.image);
    cv::waitKey(3);

    // Output modified video stream
    HSV.encoding = sensor_msgs::image_encodings::BGR8;
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "color_tracker_controller");
  ColorTrackerController ic;
  ros::spin();
  return 0;
}

