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
	pub_coord = nh_.advertise<geometry_msgs::PointStamped>(pub_topic_name ,1);

	// Create OpenCV Window to display
	OPENCV_WINDOW = pub_display;
	cv::namedWindow(OPENCV_WINDOW);

} // ObstacleDetection()



// Destructor
/*ObstacleDetection::~ObstacleDetection()
{
	cv::destroyWindow(OPENCV_WINDOW);

} // ~ObstacleDetection()*/




// Interface between ROS image messages and OpenCV Mat data structure (from ROS tutorial cv_bridge C++)
void ObstacleDetection::cv_ros_iface(const sensor_msgs::ImageConstPtr& msg)
{
	// For the timeStamp
	const sensor_msgs::ImageConstPtr& im = msg;

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
		ObstacleDetection::find_ball(cv_ptr->image, im);
		
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
void ObstacleDetection::find_ball(cv::Mat img, const sensor_msgs::ImageConstPtr& msg)
{

	Mat hsv;
	cv::GaussianBlur(img, hsv, Size(3,3), 2, 2);
	cvtColor(hsv, hsv, CV_BGR2HSV);
	
	// Filter in HSV space.
	Mat hue1, hue2;
	inRange(hsv, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), hue1);
	inRange(hsv, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), hue2); // Hue range for red colors: 160-179.
	
	cv::Mat RedBall;
	cv::addWeighted(hue1, 1.0, hue2, 1.0, 0.0, RedBall);

	
	int m_size = 2;
	Mat element = getStructuringElement(MORPH_ELLIPSE, Size(m_size+1, m_size+1), Point(m_size,m_size));
	morphologyEx(RedBall, RedBall, MORPH_OPEN, element, Point(-1,-1), 6);

	imshow(OPENCV_WINDOW, RedBall);
	waitKey(1);
	
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
		//cout<< "center = " << i << ":  [" << center[i].x << ", " << center[i].y << "]" << endl;
	}// for
	
	imshow("Detected Ball", Box);
	
	// Some additional coding to convert vector<Point2f> center into float64[2] (and filter false positives)
	if (center.size() == 0)
	{

		cout << "Not able to find the ball" << endl;

	}// if 		
	
	else if (center.size() == 1)
	{
		// Create msg
		geometry_msgs::PointStamped coordinates;
	
		coordinates.point.x = center[0].x;
		coordinates.point.y = center[0].y;

		coordinates.header.stamp = msg->header.stamp;

		pub_coord.publish(coordinates);
   		cout << "Center (x, y) = (" << center[0].x << ", " << center[0].y << ")" << endl;

	}// else if 
	
	else
	{
		// Assuming that the centers detected are due to production of serveral enclosing circles close to the ball.
		// Averaging them
		vector<float> x, y;

		// Create msg
		geometry_msgs::PointStamped coordinates;

		for( int j; j<center.size(); j++)
		{
			x.push_back(center[j].x);
			y.push_back(center[j].y);

		}// for
		
		float x_sum = std::accumulate(x.begin(), x.end(), 0.0);
		float x_avg = x_sum / x.size();
		coordinates.point.x = x_avg;	

		float y_sum = std::accumulate(y.begin(), y.end(), 0.0);
		float y_avg = y_sum / y.size();
		coordinates.point.y = y_avg;

		coordinates.header.stamp = msg->header.stamp;
		
		pub_coord.publish(coordinates);
   		cout << "Center (x, y) = (" << x_avg << ", " << y_avg << ")" << endl;

	}// else


} // find_ball()






