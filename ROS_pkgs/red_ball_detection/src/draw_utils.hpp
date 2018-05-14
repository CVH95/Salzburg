#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

void drawCross(cv::Mat &img, cv::Point center, cv::Scalar color, int size)
{
	line( img, cv::Point( center.x - size, center.y - size ), cv::Point( center.x + size, center.y + size ), color, 2, CV_AA, 0);
	line( img, cv::Point( center.x + size, center.y - size ), cv::Point( center.x - size, center.y + size ), color, 2, CV_AA, 0 );
}

void drawCrosses(cv::Mat &img, cv::Point meas, cv::Point kf1, cv::Point kf2, cv::Point kf3)
{
	drawCross(img, meas, cv::Scalar(0, 0, 255), 5);
	drawCross(img, kf1, cv::Scalar(255, 255, 255), 5);
	drawCross(img, kf2, cv::Scalar(255, 0, 0), 5);
	drawCross(img, kf3, cv::Scalar(0, 255, 0), 5);
}

cv::Mat drawTrajec(cv::Point gt, cv::Point meas, cv::Point kf1, cv::Point kf2, cv::Point kf3)
{
	static cv::Mat img(500, 1000, CV_8UC3);
	static cv::Point prevGt(0, 0), prevMeas(0, 0), prevKf1(0, 0), prevKf2(0,0), prevKf3(0,0);

	line(img, prevGt, gt, cv::Scalar(125, 125, 0), 2);
	line(img, prevMeas, meas, cv::Scalar(255, 255, 0), 1);
	line(img, prevKf1, kf1, cv::Scalar(255, 255, 255), 1);
	line(img, prevKf2, kf2, cv::Scalar(255, 0, 0), 1);
	line(img, prevKf3, kf3, cv::Scalar(0, 255, 0), 1);

	prevGt = gt; prevMeas = meas; prevKf1 = kf1; prevKf2 = kf2; prevKf3 = kf3;

	return img.clone();
}

cv::Mat drawError(cv::Point meas, cv::Point kf1, cv::Point kf2, cv::Point kf3)
{
	static cv::Mat img(300, 1000, CV_8UC3);
	static cv::Point prevMeas(0, 0), prevKf1(0, 0), prevKf2(0,0), prevKf3(0,0);

	line(img, prevMeas, meas, cv::Scalar(255, 255, 0), 1);
	line(img, prevKf1, kf1, cv::Scalar(255, 255, 255), 1);
	line(img, prevKf2, kf2, cv::Scalar(255, 0, 0), 1);
	line(img, prevKf3, kf3, cv::Scalar(0, 255, 0), 1);

	prevMeas = meas; prevKf1 = kf1; prevKf2 = kf2; prevKf3 = kf3;

	return img.clone();
}
