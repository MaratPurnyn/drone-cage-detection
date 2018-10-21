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

static const std::string OPENCV_WINDOW = "Image window";
using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;
Mat matTracking;
int lastX = -1;
int lastY = -1;
Mat frameArray[10];
int frameCount = 0;
int frameCountMinus9 = 1;
float videoStartTick = clock();
int totalFrames = 0;
string imageName = "pink1.JPG"; //default imageName
VideoWriter vidWriter;
sensor_msgs::Imu orientationData;
double dronePitch = 1000;

//// OutdoorTest Video Params
//int lowerH=2;
//int lowerS=0;
//int lowerV=65;
//int upperH=20;
//int upperS=256;
//int upperV=256;

//// IndoorTest Video Params
//int lowerH=2;
//int lowerS=0;
//int lowerV=44;
//int upperH=15;
//int upperS=181;
//int upperV=161;

// OutdoorTest Nose Video Params
int lowerH=0;
int lowerS=13;
int lowerV=20;
int upperH=11;
int upperS=256;
int upperV=162;

//// IndoorTest Nose Video Params
//int lowerH=1;
//int lowerS=74;
//int lowerV=55;
//int upperH=12;
//int upperS=256;
//int upperV=137;

// IndoorTest 2 Nose Video Params
//int lowerH=115;
//int lowerS=59;
//int lowerV=0;
//int upperH=180;
//int upperS=256;
//int upperV=256;

int distanceRes = 1;
int angleRes = CV_PI/180;
int thresholdHough = 150;
int thresholdCanny1 = 100;
int thresholdCanny2 = 200;
int dilateType = 2;
int dilateKernal = 5;

class CageDetection
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  CageDetection()
    : it_(nh_)
  {
//	  Several functions can be uncommented to perform different functions

//	  	  	 creates the windows, sliders and everything needed in the various functions.
			SetWindowSettings();

//			 cretes a image with the cage removed from a video file
			removeCage();

//			subscribes to the usb_cam video stream
//			usb_cam node must be running and the webcam must be connected to virtual machine
//			I (Peter Purnyn) had trouble with the built in webcam on this computer (Sony VIAO)
//			I used an externally connected USB webcam of my own (Logitech C615)
//			image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &CageDetection::recordVideo, this);

//			I (Peter Purnyn) never used it,
//			but the following line would create a topic to publish an output video on for another program to use
//			image_pub_ = it_.advertise("/cage_detection/output_video", 1);

  }

  ~CageDetection()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  Mat GetThresholdedImage(Mat matHSV){ // Takes an input image and outputs a mask of the threshold color
	  Mat matThresh = Mat(matHSV.size(),CV_8U);
	  inRange(matHSV, Scalar(lowerH,lowerS,lowerV), Scalar(upperH,upperS,upperV),matThresh);
	  return matThresh;
  }

  void SetWindowSettings(){
//	  should be split amongst the appropriate functions,
//	  but It's easier to be able to modify all the windows in one place
	namedWindow(OPENCV_WINDOW,WINDOW_NORMAL);
	namedWindow("Region of Interest",WINDOW_NORMAL);
  	namedWindow("Threshold",WINDOW_NORMAL);
  	namedWindow("Mask",WINDOW_NORMAL);
  }

  void FindBlobs(const Mat &binary, vector <vector<Point2i>> &blobs)
  { //This function iterates through a black and white matrix and finds all the connected white pixels and
	//labels each set of connected white pixels as a "blob" idealy a blob should be an edge of the cage
      blobs.clear();

      // Fill the label_image with the blobs
      // 0  - background
      // 1  - unlabelled foreground
      // 2+ - labelled foreground

      Mat label_image;
      binary.convertTo(label_image, CV_32SC1);

      int label_count = 2; // starts at 2 because 0,1 are used already

      for(int y=0; y < label_image.rows; y++) {
          int *row = (int*)label_image.ptr(y);
          for(int x=0; x < label_image.cols; x++) {
              if(row[x] != 1) {
                  continue;
              }

              Rect rect;
              floodFill(label_image, Point(x,y), label_count, &rect, 0, 0, 4);

              vector <Point2i> blob;

              for(int i=rect.y; i < (rect.y+rect.height); i++) {
                  int *row2 = (int*)label_image.ptr(i);
                  for(int j=rect.x; j < (rect.x+rect.width); j++) {
                      if(row2[j] != label_count) {
                          continue;
                      }

                      blob.push_back(Point2i(j,i));
                  }
              }

              blobs.push_back(blob);

              label_count++;
          }
      }
  }

  Mat removeSmallBlobs(Mat src, int sizeThreshold){
	  //finds blobs and removes them if their density is smaller than the sizeThreshold
	    Mat img;
	    cvtColor(src,img, CV_BGR2GRAY); // force greyscale

	    Mat output;
	    output = Mat::zeros(src.rows, src.cols, CV_8UC3);

	    Mat binary;
	    vector <vector<Point2i>> blobs;

	    threshold(img, binary, 0.0, 1.0, THRESH_BINARY);

	    FindBlobs(binary, blobs);
	    // Randomly color the blobs
	    for(size_t i=0; i < blobs.size(); i++) {

	        unsigned char r = 255 * (rand()/(1.0 + RAND_MAX));
	        unsigned char g = 255 * (rand()/(1.0 + RAND_MAX));
	        unsigned char b = 255 * (rand()/(1.0 + RAND_MAX));
          if(blobs[i].size() > sizeThreshold){
			for(size_t j=0; j < blobs[i].size(); j++) {
				int x = blobs[i][j].x;
				int y = blobs[i][j].y;
				output.at<Vec3b>(y,x) = Vec3b(255,255,255);
			}
          }
	    }

	    return output;
  }


  Mat connectedLabeling(Mat src){
	  //this program finds all the connected pixels and labes them as blobs
	  //it then creates a best fit line for all these blobs which follows the path of the cage
	  //a blob is hopefully an edge of the cage.
	    Mat img;
	    cvtColor(src,img, CV_BGR2GRAY); // force greyscale

	    Mat output;
	    output = Mat::zeros(src.rows, src.cols, CV_8UC3);

	    Mat binary;
	    vector <vector<Point2i>> blobs; //an array to contain all the blobs

	    threshold(img, binary, 0.0, 1.0, THRESH_BINARY);

	    FindBlobs(binary, blobs);
	    vector<Vec4f> lines; //creates an array to contain all the lines
	    Vec4f line1;
	    //next integers are used to determine length of the line segment of the best fit line
	    int bX0 = -1;
	    int bX1 = -1;
	    int bY0 = -1;
	    int bY1 = -1;

	    // Randomly color the blobs
	    for(size_t i=0; i < blobs.size(); i++) {
		    bX0 = -1;
		    bX1 = -1;
		    bY0 = -1;
		    bY1 = -1;
	        unsigned char r = 255 * (rand()/(1.0 + RAND_MAX));
	        unsigned char g = 255 * (rand()/(1.0 + RAND_MAX));
	        unsigned char b = 255 * (rand()/(1.0 + RAND_MAX));

            if(blobs[i].size()>50){ //if the density of the blob is greater than 50 we will best fit a line to it
            	fitLine(blobs[i],line1,CV_DIST_L2,0,0.01,0.01); //this functions creates a best fit line for the blob
				for(size_t j=0; j < blobs[i].size(); j++) {
					//this loop calculates the leftmost and rightmost points on a blob
					//these distance between these points + a constant are used to determine the line segment length
					// that is created from the best fit line
					int x = blobs[i][j].x;
					int y = blobs[i][j].y;

					if(bX0 == -1)
						bX0 = x;
					if(bX1 == -1)
						bX1 = x;
					if(bY0 == -1)
						bY0 = y;
					if(bY1 == -1)
						bY1 = y;
					if(bX0 > x){
						bX0 = x;
						bY0 = y;
					}
					if(bX1 < x){
						bX1 = x;
						bY1 = y;
					}

					output.at<Vec3b>(y,x)[0] = b;
					output.at<Vec3b>(y,x)[1] = g;
					output.at<Vec3b>(y,x)[2] = r;
				}
            }
            else{
            	for(size_t j=0; j < blobs[i].size(); j++) {
					int x = blobs[i][j].x;
					int y = blobs[i][j].y;
					output.at<Vec3b>(y,x) = Vec3b(0,0,0);
            	}
            }
	        double m = max(output.cols,output.rows);
	        Point2f p1(bX0,bY0);
	        Point2f p2(bX1,bY1);
	        double dis = norm(p1-p2)/2+20;
	        CvPoint startPoint;
	        startPoint.x = line1[2]- dis*line1[0];// x0
	        startPoint.y = line1[3] - dis*line1[1];// y0
	        // calculate end point
	        CvPoint endPoint;
	        endPoint.x = line1[2]+ dis*line1[0];//x[1]
	        endPoint.y = line1[3] + dis*line1[1];//y[1]

	        line(output, startPoint, endPoint, Scalar(255,255,255), 5, CV_AA); //creates the best fit line segment
	    }
	    printf ("Blobs: %d  \n", blobs.size()); //prints blob density for debugging

	    return output;
  }

  Mat dilateImage( Mat src)
  { // for information on how this works go here:
	// http://docs.opencv.org/doc/tutorials/imgproc/erosion_dilatation/erosion_dilatation.html
   Mat matDilated;
   Mat element = getStructuringElement( dilateType,
                                         Size( 2*dilateKernal + 1, 2*dilateKernal+1 ),
                                         Point( dilateKernal, dilateKernal ) );
   /// Apply the dilation operation
   dilate( src, matDilated, element );

    return matDilated;
  }

  void removeCage(){
	  //takes a video file in the videos folder and runs through it in order to remove the cage
	  printf ("camera not in use, working with video file. \n");
	  while(1==1){
		  Mat matHSV,matThresh,matMask,frame,frameModified,dst,cdst,matROI,frameResized,matResult,matResultMask;
		  float videoLoopStartTick = clock();
		  float videoDuration;
		  float timePeriod = CLOCKS_PER_SEC;
		  videoStartTick = clock();
		  frameCount = 0; //current frame
		  frameCountMinus9 = 1; //frame from 9 frames ago (due to overwriting nature of this 10 frame array)
		  printf ("Please input a video file name: \n");
		  string fileName;
		  cin >> fileName;
		  VideoCapture captRefrnc; // this will contain the video we open from the videos folder
		  captRefrnc.open("videos/"+fileName);
		  if ( !captRefrnc.isOpened()){
		    cout  << "Could not open reference " << fileName << endl;
		    return;
		    }
		  while(1==1){
			  captRefrnc >> frame; //transfers current frame of videofile to Mat type for easier editing
				if (frame.empty()){
					cout << " You've reached the end of the video file. ";
					imwrite("result_"+fileName+".jpg", matResult);
					break;
				}
			  resize(frame,frameResized,Size(),.5,.5); //resizes frame to 50% so program executes faster
			  printf ("Frame Area: %d \n", frameResized.size().area());
			  frameModified = frameResized.clone();

				if(totalFrames == 0){ //runs on functions first run
					imwrite("calibrate_"+fileName+".jpg", frameResized);
					//creates an image file where we will put our first frame for calibrating
					//the color threshold for this video, but also for comparing our result image with the cage removed
					for(int i=0;i<10;i++){ //this fils the array with the first frame so that no NULL errors result
						frameArray[i] = frameResized.clone();
					}
					totalFrames = 1;
					matResult = frameResized.clone();
				}
				else{
					totalFrames++;
				}

				videoLoopStartTick = clock();
				videoDuration = (videoLoopStartTick- videoStartTick)/timePeriod;
				printf ("Time: %2.1f  |  ", videoDuration);
				printf ("Frames: %d \n", totalFrames);

				if(frameCount < 10){ //this puts the current frame in the current position in the array
					frameArray[frameCount] = frameResized.clone();
				}

			  //blurs current frame  and also tresholds for the HSV color (color of cage)
			  Size ksize;
			  ksize.height = 3;
			  ksize.width = 3;
			  GaussianBlur(frameModified, frameModified, ksize, 0, 0);
			  matHSV = Mat(frameModified.size(), CV_8UC3);
			  cvtColor(frameModified, matHSV, CV_BGR2HSV); //Change the color format from BGR to HSV
			  matThresh = GetThresholdedImage(matHSV);
			  GaussianBlur(matThresh, matThresh, ksize, 0, 0);//smooth the binary image using Gaussian kernel
	          //used to convert the output of the thresholding into a 3channel image instead of 1
			  vector<Mat> matarray;
			  matarray.push_back(matThresh);
			  matarray.push_back(matThresh);
			  matarray.push_back(matThresh);
			  merge(matarray, matThresh);
			  matarray.empty();

			  matMask = matThresh.clone(); //for debugging purposes keep threshold result to show
			  	  	  	  	  	  	  	   //and continue working with the result
			  matMask = removeSmallBlobs(matMask, 100); // removes "blobs" of less than 100pixels in density
			  	  	  	  	  	  	  	  	  	  	  	// to remove random error
			  // expands the threshold
			  matMask = dilateImage(matMask); //uses dilation on the black and white mask to increase size of white areas

			  // finds blobs in mask and creates best fit lines to fill gaps leftover from color-thresholding
			  matMask = connectedLabeling(matMask);

			  if(totalFrames==1){
			    matResultMask = matMask.clone(); //saves the first mask and frame so that the cage may be removed from them.
			  }

			  matROI = matMask.clone();
			  frameModified.copyTo(matROI,matMask); //extracts masked area from frame to Region of Interest

			   for(int i = 0; i < matMask.rows; i++)
			   {
				   for(int j = 0; j < matMask.cols; j++)
				   { //iterates through first mask looking for cage that hasn't been replaced yet.
					 //if current frame has no mask on the pixel, then the first frame is replaced
					 //at that point. In this way subsequent frames slowly replace the parts we want replaced in the
					 //the first image
					  Vec3b pix = matMask.at<Vec3b>(i, j);
					  Vec3b pixResult = matResultMask.at<Vec3b>(i, j);
					  if (pix == Vec3b(0, 0, 0) && pixResult != Vec3b(0,0,0)) {
						  matResult.at<Vec3b>(i, j) = frameModified.at<Vec3b>(i, j);
						  matResultMask.at<Vec3b>(i,j) = Vec3b(0,0,0);
					  }
				   }
			   }

			  imshow("Threshold", matThresh);
			  imshow("Mask", matResult);
			  imshow("Region of Interest", matROI);
			  imshow(OPENCV_WINDOW, frameModified);

		  // resize windows
			  cvResizeWindow("Mask",400,300);
			  cvResizeWindow("Threshold",400,300);
			  cvResizeWindow("Image window",400,300);
			  cvResizeWindow("detected lines",400,300);
			  cvResizeWindow("Region of Interest",400,300);

		  //if escape is pressed, ask for a new image by leaving this loop.
			  int c = cvWaitKey(10);
			  if((char)c==27 ) return;

			  frameCount++;
			  frameCountMinus9++;
			  if (frameCount == 10){ //max array cell number is 9 so at 10, we reset it to 0
				  frameCount = 0;
			  }
			  if(frameCountMinus9 == 10){
				  frameCountMinus9 = 0;
			  }
		  }

		  //Clean up used images
		  matHSV.release();
		  matThresh.release();
		  matMask.release();
		  matROI.release();
		  frameModified.release();
		  dst.release();
		  cdst.release();
	  }
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cage_detection"); //creates a new ros node for this fucntion
  ros::NodeHandle n; //creates a nodehandle for this program's node, to connect it through ros
  CageDetection cd; //creates a new object in which I use image transport to subscribe to the drone camera and transport it to use with OpenCV

  ros::spin(); //This keeps the node on
  return 0; //Returns 0 if the node stops spinning.
}
