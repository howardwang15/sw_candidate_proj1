#ifndef FRAME_H
#define FRAME_H

#include <opencv2/imgproc/imgproc.hpp>

//holds data of frames
struct Frame {
	int n_frame = 0;
	int stillCount = 0;
	int validContours = 0;
	double oldContourArea; //previous frame's largest contour area
	double newContourArea; //current frame's largest contour area
	double secondLargestContourArea = 0; //stores the current frame's second largest contour
	double welderAngle; //angle of the welder path
	int n_contours; //number of contours in the current frame
	cv::Mat frame; //current frame to be processed
	cv::Mat gray; //current frame in grayscale
	cv::Point2f oldCM; //biggest contour's center of mass in the previous frame
	cv::Point2f currentCM; //biggest contour's center of mass in the current frame
};

#endif