// Stereopsis Shared Library

// ROVI2 Object Avoidance

// Richárd, Sergi, Mathesh and Carlos

// Stereopsis.h

// General 
#include <stdio.h>
#include <iostream>
#include <fstream>
//#include <array>
#include <string>
#include <vector>
// ROS
#include <ros/ros.h>
#include <red_ball_detection/ballCentrum.h> // Created msg format
#include <red_ball_detection/ballToRobotBase.h> // Created msg format
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/PointStamped.h>
// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>


using namespace std;
using namespace cv;


struct Camera {
    cv::Mat intrinsic;
    cv::Mat transformation;
    cv::Mat distortion;
    cv::Mat projection;
    cv::Mat translation;
    cv::Mat rotation;
    double image_width;
    double image_height;

    /*void printData() {
        std::cout << image_width << " " << image_height << "\n" << intrinsic << "\n"
                << distortion << "\n" << transformation << "\n" << projection
                << std::endl;
    }*/
};

struct StereoPair {
    Camera cam1;
    Camera cam2;
};


class Stereopsis{

	private:

		ros::Subscriber left_subs;
		ros::Subscriber right_subs;
		ros::Publisher pub;
		
		// Declare messages 		
		geometry_msgs::PointStamped left2D;
		geometry_msgs::PointStamped right2D;
		geometry_msgs::PointStamped location3D;

		StereoPair stPair;
		cv::Mat proj_l, proj_r;
		
		//float left0, left1;
		//float right0, right1;


	public:
		
		Stereopsis(ros::NodeHandle nh, const string pub_topic_name);

		void loadCamFromStream(ifstream & input, Camera &cam);
		bool readStereoCameraFile(const string & fileName);
		cv::Mat constructProjectionMat(Camera cam);
		void getProjectionMat();

		void calculate_3D_location(vector<float> both);
		//void broadcast_3D_location(vector<float> point);
		void synchronized_triangulation(const geometry_msgs::PointStamped::ConstPtr &image_left, const geometry_msgs::PointStamped::ConstPtr &image_right);

};
