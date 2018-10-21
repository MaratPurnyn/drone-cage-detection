#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/stitching/stitcher.hpp>
#include <opencv2/opencv.hpp>
#include <cstdio>
#include <iostream>
#include "std_msgs/String.h"
#include <opencv/cvaux.h>
#include <opencv/cxcore.h>
#include <time.h>
#include <math.h>
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/Imu.h"

//for recording video from the parrot ardrone 2.0

static const std::string OPENCV_WINDOW = "Image window";
using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;
int totalFrames = 0;
VideoWriter vidWriter;
sensor_msgs::Imu orientationData;
double dronePitch = 1000;

class RecordVideo
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  RecordVideo()
    : it_(nh_)
  {

//	  	  	 creates the windows, sliders and everything needed in the various functions.
			SetWindowSettings();

//			subscribes to the video feed from the ardrone to record it
//			ardrone must be on, this computer must be on the ardrone wifi network
//			and the ardrone_autonomy node must be up and running,
//			otherwise use the /usb_cam/image_raw line below
//			the code currently does not record the pitch values, but it would be good to write code that recorded
//			pitch values with the current frame for each recorded video.
			image_sub_ = it_.subscribe("ardrone/front/image_raw", 1, &RecordVideo::recordVideo, this);

  }

  ~RecordVideo()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void SetWindowSettings(){
	namedWindow(OPENCV_WINDOW,WINDOW_NORMAL);
  }

  void recordVideo(const sensor_msgs::ImageConstPtr& msg){
	  // record video from topic you are subscribed to
	  double pitch0 = 0; // This sets the starting pitch orientation to 0
	  	  	  	  	  	 // in case you do not subscribe to the orientation IMU topic of the ardrone
    cv_bridge::CvImagePtr cv_ptr; //This is the object that contains the image data from image transport
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    if(dronePitch != 1000){ // the pitch should only be 1000 if the IMU orientation topic is not subscribe to
    	printf ("Pitch: %2.1f  |  ",  dronePitch);
     }
    else{ //if topic was not subscribed to we will pretend the pitch is unchanging
    	dronePitch = 0;
    }

    Mat frame = cv_ptr->image.clone(); // transfers current frame to Mat datatype for easier manipulation

	if(totalFrames==0){ //the following code must only run one time on the first run of the function
		  printf ("Please input a name for the output video: \n"); //give a name for the output videofile
		  string fileName;
		  cin >> fileName;
		  pitch0 = dronePitch; //sets the starting pitch to the pitch at the first frame
		vidWriter.open(fileName+".avi",0,20,frame.size(),1); // starts a videowriter to record frames @20fps
		if(vidWriter.isOpened() == false){ //checks if the video writer has been opened correctly
			printf ("Video Writer failed to open.\n");
			return;
		}
	}

	//matShift is the transforming matrix that shifts the frame based on pitch difference from
	//starting value in order to try and match up objects in the background.
    Mat matShift = (Mat_<double>(2,3) << 1, 0, 0, 0, 1, -(pitch0-dronePitch)*200);
    warpAffine(frame,frame,matShift,frame.size());

    totalFrames++;
	printf ("Frames: %d \n", totalFrames);

	imshow(OPENCV_WINDOW, frame);
	vidWriter.write(frame);

//	Clean up used images
    frame.release();

    //	Wait 10mS
        int c = cvWaitKey(10);
    //	If 'ESC' is pressed, break the loop
        if((char)c==27 ){
        	vidWriter.release();
        	printf ("Video Writer released. Recording Completed.\n");
        	exit(-1);
        };
  }

};

void imuCallback(const sensor_msgs::Imu& orientationData){
// This function is the callback of subscribing to the ardrone
// From here you can use orientationData to access the data of the IMU provided through sensor_msgs
// I use this to determine where the drone is looking.
	//printf ("\n OrientationData \n");
	//printf ("Mag: %2.1f  |  ", orientationData.orientation.w);
	//printf ("Pitch: %2.1f  |  ", orientationData.orientation.x );
	//printf ("Roll: %2.1f  |  ", orientationData.orientation.y);
	//printf ("Yaw: %2.1f  |  ", orientationData.orientation.z);
	dronePitch = orientationData.orientation.x; //I use this global variable to access the pitch in my camera callback
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cage_detection_recordVideo"); //creates a new ros node for this fucntion
  ros::NodeHandle n; //creates a nodehandle for this program's node, to connect it through ros
  ros::Subscriber sub = n.subscribe("ardrone/imu", 1000, imuCallback); //subscribes to the ardrone/imu topic
  RecordVideo rv; //creates a new object in which I use image transport to subscribe to the drone camera and transport it to use with OpenCV

  ros::spin(); //This keeps the node on
  return 0; //Returns 0 if the node stops spinning.
}
