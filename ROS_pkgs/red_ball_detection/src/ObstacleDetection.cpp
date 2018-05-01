#include <ObstacleDetection.h>

using namespace cv;
using namespace std;


// This is the object constructor for the class ObstacleDetection. When the object is created in the main, it requires a ros::NodeHandle argument.
// See definition in detector.cpp
ObstacleDetection::ObstacleDetection(const string sub_topic_name, const string pub_topic_name, const string pub_display) : itp(nh_)
{
	
	// Subscribe to camera Image (right or left) 
	img_subscriber = itp.subscribe(sub_topic_name, 1, &ObstacleDetection::cv_ros_iface, this);

	// Create publisher to display image (for debugging, no need in the final application)
	img_publisher = itp.advertise(pub_display, 1);
	
	// Create Publisher and topic to broadcast image coordinates of the ball in the form of rosmsg [/red_ball_detection/ballCentrum]
	pub_coord = nh_.advertise<red_ball_detection::ballCentrum>(pub_topic_name ,1);

	// Hue value range for red color.
	low_h = 0;
	high_h = 15;
	kernel_size = 3;

	// Create OpenCV Window to display
	OPENCV_WINDOW = pub_display;
	cv::namedWindow(OPENCV_WINDOW);

} // ObstacleDetection()



// Destructor
ObstacleDetection::~ObstacleDetection()
{
	cv::destroyWindow(OPENCV_WINDOW);

} // ~ObstacleDetection()




// Interface between ROS image messages and OpenCV Mat data structure (from ROS tutorial cv_bridge C++)
void ObstacleDetection::cv_ros_iface(const sensor_msgs::ImageConstPtr& msg)
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

    	// Perform detection
    	if (cv_ptr->image.rows > 400 && cv_ptr->image.cols > 600)
	{

		// Call to the detection function
		ObstacleDetection::find_ball(cv_ptr->image);
		
		// Publish thresholded image (debugging)
    		img_publisher.publish(cv_ptr->toImageMsg());

	} // if

} // cv_ros_iface()


//---------------------------------------------------------


// Functions to create the trackbar for adjustment of detection thresholds.
/*void ObstacleDetection::track_low_h(int,void*)
{
	setTrackbarPos("min_h", OPENCV_WINDOW, low_h);

}// track_low_h() 

void ObstacleDetection::track_high_h(int,void*)
{
	setTrackbarPos("max_h", OPENCV_WINDOW, high_h);

}// track_high_h()*/


//---------------------------------------------------------------



// Function to detect the red ball in the scene
void ObstacleDetection::find_ball(cv::Mat img)
{

	Mat hsv;
	cv::GaussianBlur(img, hsv, Size(3,3), 2, 2);
	cvtColor(hsv, hsv, CV_BGR2HSV);
	
	// Split image into H-S-V channels.
	Mat planes[3];
	split(hsv, planes);
	
	// Filter in Hue plane.
	Mat RedBall;
	inRange(planes[0], low_h, high_h, RedBall); // Thresholding Function. Result: Binary image.
	
	//createTrackbar("min_H", OPENCV_WINDOW, &low_h, 255, ObstacleDetection::track_low_h);
	//createTrackbar("max_H", OPENCV_WINDOW, &high_h, 255, ObstacleDetection::track_high_h);
 
	imshow(OPENCV_WINDOW, RedBall);
	waitKey(1);
	
	int m_size = 2;
	Mat element = getStructuringElement(MORPH_ELLIPSE, Size(m_size+1, m_size+1), Point(m_size,m_size));
	morphologyEx(RedBall,RedBall, MORPH_OPEN, element,Point(-1,-1),6);
	
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(RedBall, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));
		
	vector<vector<Point> > contours_poly(contours.size());
	vector<Rect> boundRect (contours.size());
	vector<Point2f> center(contours.size());
	vector<float> radius(contours.size());
  	
  	for(int i=0; i<contours.size(); i++)
	{
		approxPolyDP(contours[i],contours_poly[i],5,true);
		boundRect[i] = boundingRect(contours_poly[i]);
		minEnclosingCircle(contours_poly[i],center[i],radius[i]);
	}// for

	Mat Box = Mat::zeros(RedBall.size(), CV_8UC3);
	for(int i=0; i< contours.size(); i++)
	{
		drawContours(Box, contours, -1, Scalar(0,0,255),1);
		rectangle(Box, boundRect[i], Scalar(0,255,255), 1,8,0);
		circle(Box,center[i],radius[i], Scalar(255,0,0),1,8,0);
		cout<< "center = " << i << ":  [" << center[i].x << ", " << center[i].y << "]" << endl;
	}// for
	
	//imshow("Detected Ball", Box);
	
	// Some additional coding to convert vector<Point2f> center into float64[2] (and filter false positives)
	if (center.size() == 0)
	{

		cout << "Not able to find the ball" << endl;

	}// if 		
	
	else if (center.size() == 1)
	{
		float coordinates[2];	
		coordinates[0] = center[0].x;
		coordinates[1] = center[0].y;

		pub_coord.publish(coordinates[2]);
   		cout << "Center (x, y) = (" << coordinates[0] << ", " << coordinates[1] << ")" << endl;

	}// else if 
	
	else
	{
		// Assuming that the centers detected are due to production of serveral enclosing circles close to the ball.
		// Averaging them
		vector<float> x, y;
		float coordinates[2];
		for( int j; j<center.size(); j++)
		{
			x.push_back(center[j].x);
			y.push_back(center[j].y);

		}// for
		
		float x_sum = std::accumulate(x.begin(), x.end(), 0.0);
		float x_avg = x_sum / x.size();
		coordinates[0] = x_avg;	

		float y_sum = std::accumulate(y.begin(), y.end(), 0.0);
		float y_avg = y_sum / y.size();
		coordinates[1] = y_avg;
		
		pub_coord.publish(coordinates[2]);
   		cout << "Center (x, y) = (" << coordinates[0] << ", " << coordinates[1] << ")" << endl;

	}// else


} // find_ball()






