#ifndef VIDEO_H
#define VIDEO_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "Frame.h"

//video processing class
class Video {
	public:
		//class invariant: fileName is a valid file path to the video
		Video(std::string fileName); 
		void display(); //displays the video frame by frame with processing
		void toGray(Frame& frame); //converts each frame to grayscale
		void resize(cv::Mat& frame, double factor); //resizes the frame 
		void threshold(cv::Mat& frame, int low); //converts grayscale image into binary image
		void drawHull(Frame& frame, cv::Mat& original, std::vector<int>& skippedFrames); //detects the weld pool
		bool dropped(const cv::Mat& previous, const cv::Mat& current, Frame& frame);
		void calcWelderAngle(Frame& frame); //Calculates the welder incline
		int isStill(const cv::Mat& previous, const cv::Mat& current, Frame& frame); //determines whether or not there was motion in between 2 frames

	private:
		cv::VideoCapture m_cap;
};
#endif