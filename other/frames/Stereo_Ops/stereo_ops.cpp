#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>


using namespace cv;
using namespace std;

Mat hsv_l,hsv_r;

int low_h=0,high_h=255;

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
	
	VideoCapture cap0(0);
	VideoCapture cap1(1);
	if(!cap0.isOpened())
	{
		cout<<"Nothing Happened"<<endl;
		return -1;
	}
	
	
	
	while(1)
	{
		Mat frames_l,frames_r;
		cap0>>frames_l;
		cap1>>frames_r;
		imshow("Stereo_Left",frames_l);
		imshow("Stereo_Right",frames_r);
		int w = cap0.get(CV_CAP_PROP_FRAME_WIDTH);
		int h = cap0.get(CV_CAP_PROP_FRAME_HEIGHT);
		VideoWriter("original_video.avi",CV_FOURCC('M','J','P','G'),10,Size(w,h),false);
		waitKey(1);
		if((char)waitKey(10)==27)
			break;
		GaussianBlur(frames_l,frames_l,Size(3,3),2,2);
		GaussianBlur(frames_r,frames_r,Size(3,3),2,2);		
		
		cvtColor(frames_l,hsv_l,CV_BGR2HSV);
		cvtColor(frames_r,hsv_r,CV_BGR2HSV);		
	
		Mat RedBall_l,RedBall_r;
		inRange(hsv_l,Scalar(low_h,120,120),Scalar(high_h,255,255),RedBall_l);
		inRange(hsv_r,Scalar(low_h,120,120),Scalar(high_h,255,255),RedBall_r);
		
		//namedWindow("HSV",WINDOW_NORMAL);
		
		createTrackbar("min_H","RedBall_Left",&low_h,255,track_low_h);
		createTrackbar("max_H","RedBall_Left",&high_h,255,track_high_h);
		
		createTrackbar("min_H","RedBall_Right",&low_h,255,track_low_h);
		createTrackbar("max_H","RedBall_Right",&high_h,255,track_high_h);

		//imshow("HSV",RedBall);
		//waitKey(1);
		int m_size = 2;
		Mat element = getStructuringElement(MORPH_ELLIPSE, Size(m_size+1, m_size+1), Point(m_size,m_size));
		morphologyEx(RedBall_l,RedBall_l, MORPH_OPEN, element,Point(-1,-1),6);
		morphologyEx(RedBall_r,RedBall_r, MORPH_OPEN, element,Point(-1,-1),6);
	
		//namedWindow("RedBall",WINDOW_NORMAL);
		imshow("RedBall_Left",RedBall_l);
		imshow("RedBall_Right",RedBall_r);
		
		vector<vector<Point> > contours_l, contours_r;
		vector<Vec4i> hierarchy_l, hierarchy_r;
		findContours(RedBall_l, contours_l, hierarchy_l, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));
		findContours(RedBall_r, contours_r, hierarchy_r, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));

		vector<vector<Point> > contours_poly_l(contours_l.size()),contours_poly_r(contours_r.size());
		vector<Rect> boundRect_l(contours_l.size()),boundRect_r(contours_r.size());
		vector<Point2f> center_l(contours_l.size()),center_r(contours_r.size());
		vector<float> radius_l(contours_l.size()),radius_r(contours_r.size());
  	
  		for(int i=0; i<contours_l.size(); i++)
		{
			approxPolyDP(contours_l[i],contours_poly_l[i],5,true);
			boundRect_l[i] = boundingRect(contours_poly_l[i]);
			minEnclosingCircle(contours_poly_l[i],center_l[i],radius_l[i]);
		}
		
		for(int i=0; i<contours_r.size(); i++)
		{
			approxPolyDP(contours_r[i],contours_poly_r[i],5,true);
			boundRect_r[i] = boundingRect(contours_poly_r[i]);
			minEnclosingCircle(contours_poly_r[i],center_r[i],radius_r[i]);
		}

		Mat Box_l = Mat::zeros(RedBall_l.size(), CV_8UC3);
		Mat Box_r = Mat::zeros(RedBall_r.size(), CV_8UC3);

		for(int i=0; i< contours_l.size(); i++)
		{
			drawContours(Box_l, contours_l, -1, Scalar(0,0,255),1);
			rectangle(Box_l, boundRect_l[i], Scalar(0,255,255), 1,8,0);
			circle(Box_l,center_l[i],radius_l[i], Scalar(255,0,0),1,8,0);
			//cout<< "center = " << center[i]<< endl;
		}
	
		for(int i=0; i< contours_r.size(); i++)
		{
			drawContours(Box_r, contours_r, -1, Scalar(0,0,255),1);
			rectangle(Box_r, boundRect_r[i], Scalar(0,255,255), 1,8,0);
			circle(Box_r,center_r[i],radius_r[i], Scalar(255,0,0),1,8,0);
			//cout<< "center = " << center[i]<< endl;
		}
		cout<<"Center in left = "<< center_l << endl;
		cout<<"Center in right = "<< center_r << endl;
		//namedWindow("BoundingBox", WINDOW_NORMAL);
		imshow("BoundingBox-Left",Box_l);
		imshow("BoundingBox-Right",Box_r);		
		waitKey(1);
	}
	return 0;  
}
