// Stereopsis Shared Library

// ROVI2 Object Avoidance

// Richárd, Sergi, Mathesh and Carlos

#include <Stereopsis.h>

using namespace std;
using namespace cv;

// This is the object constructor for the class Stereopsis. When the object is created in the main, it requires a ros::NodeHandle argument.
// See definition in triangulation.cpp
Stereopsis::Stereopsis(ros::NodeHandle nh, const string pub_topic_name)
{		
	// Publisher	
	pub = nh.advertise<geometry_msgs::PointStamped>(pub_topic_name, 1);

}// Stereopsis()

//--------------------------------------------------------------------------------------------------------


// Calibration-related part


// Builds Camera structure
void Stereopsis::loadCamFromStream(ifstream & input, Camera &cam)
{

	cv::Mat intrinsic = cv::Mat::zeros(3, 3, CV_64F);
    	cv::Mat distortion = cv::Mat::zeros(4, 1, CV_64F);
    	cv::Mat projection = cv::Mat::zeros(3, 4, CV_64F);
   	cv::Mat transformation = cv::Mat::eye(4, 4, CV_64F);
    	cv::Mat translation = cv::Mat::zeros(3, 1, CV_64F);
    	cv::Mat rotation = cv::Mat::zeros(3, 3, CV_64F);
    	double image_width, image_height;
    	input.precision(20);
    	input >> image_width >> image_height;

    	input >> intrinsic.at<double>(0, 0) >> intrinsic.at<double>(0, 1)
            >> intrinsic.at<double>(0, 2);
    	input >> intrinsic.at<double>(1, 0) >> intrinsic.at<double>(1, 1)
            >> intrinsic.at<double>(1, 2);
   	input >> intrinsic.at<double>(2, 0) >> intrinsic.at<double>(2, 1)
            >> intrinsic.at<double>(2, 2);

    	input >> distortion.at<double>(0, 0) >> distortion.at<double>(1, 0)
            >> distortion.at<double>(2, 0) >> distortion.at<double>(3, 0);

    	/*input >> projection.at<double>(0, 0) >> projection.at<double>(0, 1)
            >> projection.at<double>(0, 2) >> projection.at<double>(0, 3);
    	input >> projection.at<double>(1, 0) >> projection.at<double>(1, 1)
            >> projection.at<double>(1, 2) >> projection.at<double>(1, 3);
    	input >> projection.at<double>(2, 0) >> projection.at<double>(2, 1)
            >> projection.at<double>(2, 2) >> projection.at<double>(2, 3);*/

    	input >> rotation.at<double>(0, 0) >> rotation.at<double>(0, 1)
            >> rotation.at<double>(0, 2);
    	input >> rotation.at<double>(1, 0) >> rotation.at<double>(1, 1)
            >> rotation.at<double>(1, 2);
   	input >> rotation.at<double>(2, 0) >> rotation.at<double>(2, 1)
            >> rotation.at<double>(2, 2);
    	input >> translation.at<double>(0) >> translation.at<double>(1)
            >> translation.at<double>(2);

    	cv::hconcat(rotation, translation, transformation);
    	cv::Mat row = cv::Mat::zeros(1, 4, CV_64F);
    	row.at<double>(0, 3) = 1;
    	transformation.push_back(row);

    	cv::Mat tmp = intrinsic;
    	cv::Mat tmp1 = cv::Mat::zeros(3, 1, CV_64F);
    	cv::hconcat(tmp, tmp1, tmp);
    	projection = tmp * transformation;

    	cam.distortion = distortion;
    	cam.intrinsic = intrinsic;
    	cam.projection = projection;
    	cam.transformation = transformation;
    	cam.image_height = image_height;
    	cam.image_width = image_width;
    	cam.translation = translation;
    	cam.rotation = rotation;

}// loadCamFromStream()


// Reads Calibration file
bool Stereopsis::readStereoCameraFile(const string & fileName)
{

	int number_of_cameras;
    	Camera cam1, cam2;
    	std::ifstream ifs(fileName.c_str());
    	if (ifs) 
	{

        	ifs >> number_of_cameras;
        	if (number_of_cameras == 2) 
		{

			Stereopsis::loadCamFromStream(ifs, cam1);
            		Stereopsis::loadCamFromStream(ifs, cam2);
            		stPair.cam1 = cam1;
            		stPair.cam2 = cam2;
            		return true;

        	}// if number_of_cameras

    	}// if ifs

	return false;

}// readStereoCameraFile()



//-------------------------------------------------------------------------------------------------

// Calculate Camera projection Matrix
cv::Mat Stereopsis::constructProjectionMat(Camera cam)
{
    cv::Mat KA = cam.intrinsic;
    cv::Mat H = cam.transformation;

    // Remember to add a row of zeros so the KA matrix becomes 3x4
    cv::Mat zeros = cv::Mat::zeros(3, 1, CV_64F);
    cv::hconcat(KA, zeros, KA);

    return KA * H;

}// constructProjectionMat()


void Stereopsis::getProjectionMat()
{

	proj_l = Stereopsis::constructProjectionMat(stPair.cam1);
	proj_r = Stereopsis::constructProjectionMat(stPair.cam2);

}// getProjectMat()



// Compute the 3D location of the Ball (Using OpenCV triangulation).
void Stereopsis::calculate_3D_location(vector<float> both)
{
	vector<float> location;

	cv::Mat pnts3D(1, 1, CV_64FC4);
    	cv::Mat cam0pnts(1, 1, CV_64FC2);
    	cv::Mat cam1pnts(1, 1, CV_64FC2);
    	cam0pnts.at<cv::Vec2d>(0)[0] = both[0];
    	cam0pnts.at<cv::Vec2d>(0)[1] = both[1];
    	cam1pnts.at<cv::Vec2d>(0)[0] = both[2];
    	cam1pnts.at<cv::Vec2d>(0)[1] = both[3];

	triangulatePoints(proj_l, proj_r, cam0pnts, cam1pnts, pnts3D);

	float x = (float) pnts3D.at<double>(0, 0) / pnts3D.at<double>(3, 0);
	float y = (float) pnts3D.at<double>(1, 0) / pnts3D.at<double>(3, 0);
	float z = (float) pnts3D.at<double>(2, 0) / pnts3D.at<double>(3, 0);

	// Broadcast triangulated points into the ROS network
	
	location3D.point.x = x;
	location3D.point.y = y;
	location3D.point.z = z;

	pub.publish(Stereopsis::location3D);
	cout << "x = " << x << ",  y = " << y << ",  z = " << z << endl;

}//calculate_3D_location()



/*// Call this function at the end to broadcast triangulated points into the ROS network
void Stereopsis::broadcast_3D_location(vector<float> point)
{

	//red_ball_detection::ballToRobotBase msg;
	
	pose_3D.data.push_back(point[0]);
	pose_3D.data.push_back(point[1]);
	pose_3D.data.push_back(point[2]);
	
	// Define timeStamp to synchronize publishing
	pose_3D.header.stamp = ros::Time::now();

	pub.publish(pose_3D);

}// broadcast_3D_location()*/


// Function that gets synchronized msgs from detectors and performs triangulation + broadcast
void Stereopsis::synchronized_triangulation(const geometry_msgs::PointStamped::ConstPtr &image_left, const geometry_msgs::PointStamped::ConstPtr &image_right)
{
	left2D = *image_left;
	right2D = *image_right;

	vector<float> both;
	both.push_back(left2D.point.x);
	both.push_back(left2D.point.y);
	both.push_back(right2D.point.x);
	both.push_back(right2D.point.y);
	
	location3D.header.stamp = left2D.header.stamp;

	Stereopsis::calculate_3D_location(both);
	
}// synchronized_triangulation()
