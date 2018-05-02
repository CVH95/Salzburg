// Stereopsis Shared Library

// ROVI2 Object Avoidance

// Richárd. Sergi, Mathesh and Carlos

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
		vector<float> left;
		vector<float> right;
		//cv::Mat proj_l;
		//cv::Mat proj_r;

	public:
		Stereopsis(ros::NodeHandle nh, const string pub_topic_name);
		void get_left_coordinates(const red_ball_detection::ballCentrum msg);
		void get_right_coordinates(const red_ball_detection::ballCentrum msg);
		vector<float> group_coordinates(ros::NodeHandle nh, const string sub_left, const string sub_right);

		void loadCamFromStream(ifstream & input, Camera &cam);
		bool readStereoCameraFile(const string & fileName, StereoPair &stereoPair);
		cv::Mat constructProjectionMat(Camera cam);

		vector<float> calculate_3D_location(vector<float> both, Mat proj_l, Mat proj_r);
		void broadcast_3D_location(vector<float> point);

};
