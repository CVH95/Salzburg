#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>


using namespace cv;
using namespace std;

Mat img,src,hsv;

//int lowThreshold=30;
int kernel_size = 3;
int low_h = 0,high_h=255;
//void track_low_r(int,void*);

void track_low_h(int,void*)
{
	setTrackbarPos("min_h","RedBall",low_h);
}

void track_high_h(int,void*)
{
	setTrackbarPos("max_h","RedBall",high_h);
}

int main( int argc, char** argv )
{
	
	VideoCapture cap(0);
	if(!cap.isOpened())
	{
		cout<<"Nothing Happened"<<endl;
		return -1;
	}
	
	
	
	while(1)
	{
		Mat frames;
		cap>>frames;
		imshow("Video",frames);
		int w = cap.get(CV_CAP_PROP_FRAME_WIDTH);
		int h = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
		VideoWriter("original_video.avi",CV_FOURCC('M','J','P','G'),10,Size(w,h),false);
		waitKey(1);
		if((char)waitKey(10)==27)
			break;
		GaussianBlur(frames,frames,Size(3,3),2,2);
		cvtColor(frames,hsv,CV_BGR2HSV);
	
		Mat RedBall;
		inRange(hsv,Scalar(low_h,120,120),Scalar(high_h,255,255),RedBall);
		//inRange(hsv,Scalar(238,200,90),Scalar(240,240,120),RedBall);
		
		namedWindow("HSV",WINDOW_NORMAL);
		
		createTrackbar("min_H","RedBall",&low_h,255,track_low_h);
		createTrackbar("max_H","RedBall",&high_h,255,track_high_h);
	
		imshow("HSV",RedBall);
		waitKey(1);
		int m_size = 2;
		Mat element = getStructuringElement(MORPH_ELLIPSE, Size(m_size+1, m_size+1), Point(m_size,m_size));
		morphologyEx(RedBall,RedBall, MORPH_OPEN, element,Point(-1,-1),6);
	
		//namedWindow("RedBall",WINDOW_NORMAL);
		imshow("RedBall",RedBall);

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
		}

		Mat Box = Mat::zeros(RedBall.size(), CV_8UC3);
		for(int i=0; i< contours.size(); i++)
		{
			drawContours(Box, contours, -1, Scalar(0,0,255),1);
			rectangle(Box, boundRect[i], Scalar(0,255,255), 1,8,0);
			circle(Box,center[i],radius[i], Scalar(255,0,0),1,8,0);
			cout<< "center = " << center[i]<< endl;
		}
	
		//namedWindow("BoundingBox", WINDOW_NORMAL);
		imshow("BoundingBox",Box);
		waitKey(1);
	}
	return 0;  
}
