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

// this is for more easily determining which
// values work best for the image to detect objects
// these values are then used in cage_detection 

static const std::string OPENCV_WINDOW = "Image window";
using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;
Mat frameArray[10];
int frameCount = 0;
int frameCountMinus9 = 1;
float videoStartTick = clock();
int totalFrames = 0;

int lowerH=0;
int lowerS=13;
int lowerV=20;
int upperH=11;
int upperS=256;
int upperV=162;

int distanceRes = 1;
int angleRes = CV_PI/180;
int thresholdHough = 150;
int thresholdCanny1 = 100;
int thresholdCanny2 = 200;
int dilateType = 2;
int dilateKernal = 5;

class Calibration
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  Calibration()
    : it_(nh_)
  {
//	  	  	 creates the windows, sliders and everything needed in the various functions.
			SetWindowSettings();

//			uncomment to calibrate global variables based off single image
//			you select an image to work with and move the sliders to determine the best values for the image
//			write them down and then add them to the cage_detection.cpp
			calibrateVariables();

  }

  ~Calibration()
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
  	namedWindow("DilatedImageSliders",WINDOW_NORMAL);
  	createTrackbar("Type", "DilatedImageSliders", &dilateType, 2, NULL);
  	createTrackbar("Kernel size", "DilatedImageSliders", &dilateKernal, 20, NULL);
  	namedWindow("Sliders",WINDOW_NORMAL);
  	createTrackbar("LowerH", "Sliders", &lowerH, 180, NULL);
  	createTrackbar("UpperH", "Sliders", &upperH, 180, NULL);
  	createTrackbar("LowerS", "Sliders", &lowerS, 256, NULL);
  	createTrackbar("UpperS", "Sliders", &upperS, 256, NULL);
  	createTrackbar("LowerV", "Sliders", &lowerV, 256, NULL);
  	createTrackbar("UpperV", "Sliders", &upperV, 256, NULL);
  	namedWindow("HoughSliders",WINDOW_NORMAL);
  	createTrackbar("threshold", "HoughSliders", &thresholdHough, 300, NULL);
  	createTrackbar("rho", "HoughSliders", &distanceRes, 100, NULL);
  	createTrackbar("theta", "HoughSliders", &angleRes, CV_PI/2, NULL);
  	namedWindow("CannySliders",WINDOW_NORMAL);
  	createTrackbar("Threshold 1", "CannySliders", &thresholdCanny1, 300, NULL);
  	createTrackbar("Threshold 2", "CannySliders", &thresholdCanny2, 300, NULL);
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

  void calibrateVariables(){
	  // this function is used to calibrate the variables used in other functions
	  // it takes a single image as an input in the images folder
	  // it then gives you several sliders and outputs the result of different values
	  // this allows you to figure out the best HSV values and other global variables.
	  printf ("camera not in use, working with images. \n");
	  while(1==1){
		  Mat matHSV,matThresh,matMask,frame,frameModified,dst,cdst,matROI;
		  printf ("Please input a image file name: \n");
		  string imageName2;
		  cin >> imageName2;
		  frame = imread("images/"+imageName2);
		  resize(frame,frameModified,Size(),1,1);
		  while(1==1){
			  resize(frame,frameModified,Size(),1,1);
			  //  current frame
			  Size ksize;
			  ksize.height = 3;
			  ksize.width = 3;
			  GaussianBlur(frameModified, frameModified, ksize, 0, 0);
			  matHSV = Mat(frameModified.size(), CV_8UC3);
			  cvtColor(frameModified, matHSV, CV_BGR2HSV); //Change the color format from BGR to HSV
			  matThresh = GetThresholdedImage(matHSV);
			  GaussianBlur(matThresh, matThresh, ksize, 0, 0);//smooth the binary image using Gaussian kernel
			  vector<Mat> matarray;
			  matarray.push_back(matThresh);
			  matarray.push_back(matThresh);
			  matarray.push_back(matThresh);
			  merge(matarray, matThresh);
			  matarray.empty();


			  matMask = matThresh.clone();
			  matROI = matMask.clone();

			  matMask = removeSmallBlobs(matMask, 100);

			  // expands the threshold
			  matMask = dilateImage(matMask);

			  // finds blobs in mask and creates best fit lines to fill gaps leftover from color-thresholding
			  matMask = connectedLabeling(matMask);

			  frameModified.copyTo(matROI,matMask);

			  imshow("Threshold", matThresh);
			  imshow("Mask", matMask);
			  imshow("Region of Interest", matROI);
			  imshow(OPENCV_WINDOW, frameModified);

		  // resize windows
			  cvResizeWindow("Mask",400,300);
			  cvResizeWindow("Threshold",400,300);
			  cvResizeWindow("Image window",400,300);
			  cvResizeWindow("Region of Interest",400,300);

		  //if escape is pressed, ask for a new image by leaving this loop.
			  int c = cvWaitKey(10);
			  if((char)c==27 ) return;
		  }
		  //Clean up used images
		  matHSV.release();
		  matThresh.release();
		  matMask.release();
		  frameModified.release();
		  dst.release();
		  cdst.release();
		  matROI.release();
	  }
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cage_detection_calibration"); //creates a new ros node for this fucntion
  ros::NodeHandle n; //creates a nodehandle for this program's node, to connect it through ros
  Calibration cal; //creates a new object in which I use image transport to subscribe to the drone camera and transport it to use with OpenCV

  ros::spin(); //This keeps the node on
  return 0; //Returns 0 if the node stops spinning.
}
