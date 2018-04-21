// SAMPLE CODE FOR FEATURE EXTRACTIOM COMBINING ROS AND OPENCV


#include <ros/ros.h>
// This pkg is for publishig and subscribing images in ROS
#include <image_transport/image_transport.h>
// This headers include the CvBridge class and image encoding related functions in the code
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
// Image processing
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
// Other general headers
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <string>
#include <numeric>

using namespace cv;
using namespace std;

# define M_PI           3.14159265358979323846

// Output file
ofstream fs;

// Displaying windows
static const std::string OPENCV_WINDOW = "Input Camera View";
static const std::string OPENCV_WINDOW_1 = "Segmented View";



//--------------------------------------------------------------------------------------------


class feature_detector
{
  	ros::NodeHandle nh_;

	/* ImageTransport --> used to publish and subscribe the ROS image messages. */

  	image_transport::ImageTransport it_;

	// Create instance of the ImageTransport class
	image_transport::Subscriber image_sub_;
  	image_transport::Publisher image_pub_;
  
  public:

	// Main constructor of feature detector
  	feature_detector()
  	   : it_(nh_)
  	{
    		// Declare the SUBSCRIBER and PUBLISHER objects:
    		image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, 
      		&feature_detector::imageCb, this);

    		image_pub_ = it_.advertise("/feature_detector/processed_image", 1);
    		cv::namedWindow(OPENCV_WINDOW);
  	}
	
	// Destructor
  	~feature_detector()
  	{
    		cv::destroyWindow(OPENCV_WINDOW);
  	}

	// Image call-back function to convert ROS messages to cv::Mat data type.
  	void imageCb(const sensor_msgs::ImageConstPtr& msg)
  	{

    		cv_bridge::CvImagePtr cv_ptr;
    		namespace enc = sensor_msgs::image_encodings;

    		try
    		{
      			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    		} // try
    	
		catch (cv_bridge::Exception& e)
    		{
      			ROS_ERROR("cv_bridge exception: %s", e.what());
      			return;

    		} // catch

    		// Draw an example circle on the video stream
    		if (cv_ptr->image.rows > 400 && cv_ptr->image.cols > 600)
		{
			extract_features(cv_ptr->image);
    			image_pub_.publish(cv_ptr->toImageMsg());
		} // if

  	} // imagecb

	// Subroutine to get features
  	void extract_features(cv::Mat img)
  	{
		// Open writing file
		fs.open("/home/charlie/catkin_ws/features_tutorial/genfiles/center_points.csv");

   		cv::Mat src, src_gray;
		cv::Mat dst, detected_edges;

		int edgeThresh = 1;
		int lowThreshold = 200;
		int highThreshold =300;
		int kernel_size = 5;

		img.copyTo(src);

		cv::cvtColor( img, src_gray, CV_BGR2GRAY );
        	cv::blur( src_gray, detected_edges, cv::Size(5,5) );
		cv::Canny( detected_edges, detected_edges, lowThreshold, highThreshold, kernel_size );

		// Find contours
		vector<vector<Point> > contours;
  		vector<Vec4i> hierarchy;

		findContours(detected_edges, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );

		//----------------------------------------------------------------

		// Approximate contours to circles
		int imax = contours.size();
  		vector<vector<Point> > contours_cc(contours.size());
  		vector<Point2f>centers(contours.size());
  		vector<float>radius(contours.size());

  		for( int i = 0; i < imax; i++ )
     		{ 
			approxPolyDP( Mat(contours[i]), contours_cc[i], 3, true );
	      	 	minEnclosingCircle( (Mat)contours_cc[i], centers[i], radius[i] );
			fs << i << "," << centers[i].x << "," << centers[i].y << endl;
			cout << "Center Point " << i << ":  [" << centers[i].x << ", " << centers[i].y << "]" << endl;
			circle( detected_edges, centers[i], (int)radius[i], Scalar(0,0,255), 6, 8, 0 );
		}

		cout << endl;
		cout << endl;

		//---------------------------------------------------------------

		/*// DRAWING DETECTED CIRCLES
		for( int j = 0; j< imax; j++ )
     		{
			// Drawing 
      			circle( detected_edges, centers[j], (int)radius[j], Scalar(0,0,255), 6, 8, 0 );
     		}

		//--------------------------------------------------------------

		*/// Display
		dst = cv::Scalar::all(0);
 	 	img.copyTo( dst, detected_edges);
		dst.copyTo(img);
	
 	   	cv::imshow(OPENCV_WINDOW, src);
 	   	cv::imshow(OPENCV_WINDOW_1, dst);
 	   	cv::waitKey(3);

		fs.close();
 	} // extract_features
 
}; // class 

//--------------------------------------------------------------------------------------

int main(int argc, char** argv)
{
  	ros::init(argc, argv, "feature_detector");
  	feature_detector ic;
  	ros::spin();
  	return 0;

} // main
