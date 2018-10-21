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

// class for sticting together frames of a video 
// outputs a panorama image using the sticher class

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;
int totalFrames = 0;

class PanoramaGenerator
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  PanoramaGenerator()
    : it_(nh_)
  {
//	  	  	 creates the windows, sliders and everything needed in the various functions.
			SetWindowSettings();

//			creates a panorama from a videofile
// 			it takes images 30 frames apart and runs in through the openCV sticher class
//			to stich them together
			imagePanorama();

  }

  ~PanoramaGenerator()
  {
    cv::destroyWindow("Panorama Result");
  }

  void SetWindowSettings(){
  	namedWindow("Panorama Result",WINDOW_NORMAL);
  }

  void imagePanorama(){
	  printf ("camera not in use, creating array of images from video file. \n");
	  	  while(1==1){
	  		  Mat frame,frameModified,matResult;
	  		  Stitcher stitcher = Stitcher::createDefault();
	  		  string result_name = "result.jpg";
	  		  vector<Mat> panoArray;
	  		  printf ("Please input video file name: \n");
	  		  string fileName;
	  		  cin >> fileName;
	  		  VideoCapture captRefrnc;
	  		  captRefrnc.open("videos/"+fileName);
	  		  if ( !captRefrnc.isOpened())
	  		    {
	  		    cout  << "Could not open reference " << fileName << endl;
	  		    return;
	  		    }
	  		  while(1==1){
	  			  captRefrnc >> frame;

//	  			  resize(frame,frameResized,Size(),.3,.3);
	  			  frameModified = frame.clone();

	  				if(totalFrames == 0){
	  					totalFrames = 1;
	  				}
	  				else{
	  					totalFrames++;
	  				}
	  				if(totalFrames == 1 || totalFrames % 30 == 0){
	  					panoArray.push_back(frameModified.clone());
	  					printf ("Added frame to array. \n");
	  				}
	  				if (frame.empty())
	  				{
	  					printf ("End of the video file. Creating Panorama. \n");
	  					Stitcher::Status status = stitcher.stitch(panoArray, matResult);
	  					if (status != Stitcher::OK){
	  						cout << "Can't stitch images, error code = " << status << endl;
						}

						while(1 == 1){
							imshow("Panorama Result", matResult);
							int c = cvWaitKey(10);
							if((char)c==27 ) return;
						}

	  				}
	  		  }

	  		  //Clean up used images
	  		  frameModified.release();
	  		  frame.release();
	  		  matResult.release();
	  	  }
  }

};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "cage_detection_panorama"); //creates a new ros node for this fucntion
  ros::NodeHandle n; //creates a nodehandle for this program's node, to connect it through ros
  PanoramaGenerator pg; //creates a new object in which I use image transport to subscribe to the drone camera and transport it to use with OpenCV

  ros::spin(); //This keeps the node on
  return 0; //Returns 0 if the node stops spinning.
}
