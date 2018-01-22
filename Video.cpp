#include "stdafx.h"
#include "Video.h"
#include "Frame.h"
#include "Globals.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <opencv2/imgproc/imgproc.hpp>


//still frames(thresh = 200) : 5, 15, 24, 34, 43, 44?, 45, 53, 57, 62, 72, 81, 91, 100, 110, 119, 122, 129, 132, 138, 140, 141, 146, 148
//welder is off: 514-517 , 1238-1239, should be @16 sec, @48-51 sec 
//skipping frames: 71, 80, 81, 143-147	
//transitions: 1500

using namespace std;
using namespace cv;

Video::Video(string fileName) : m_cap(fileName) { //initialize the videocapture object in the constructor
}

//360x640 images (after resizing)
void Video::display() {

	//stores the frames of each event and writes to file after the entire video is gone through to save time
	vector<int> stillFrames;
	vector<int> welderOffFrames;
	vector<int> skippedFrames;
	vector<double> welderAngles;
	ofstream output("events.txt"); //creates the file to be written to

	Mat previous; //previous frame
	Mat original; //current frame
	Frame data; //stores info about the each

	//go through video
	while (m_cap.isOpened()) {
		cout << "frame number: " << data.n_frame << endl; //print the current frame number
		
		m_cap >> data.frame; //store current frame
		/*if (data.n_frame <= 1200) {
			data.n_frame++;
			continue;
		}*/

		if (data.frame.empty()) //quit if the video finishes
			break;

		original = data.frame.clone(); //store the original frame captured
		resize(original, 0.5);  //resize frames to make processing quicker
		resize(data.frame, 0.5); 

		toGray(data); //convert the original frame to grayscale
		threshold(data.frame, THRESH_VALUE); //convert grayscale image to binary imgae
		calcWelderAngle(data); //calculate the welder incline in the frame
		welderAngles.push_back(data.welderAngle);
		drawHull(data, original, skippedFrames); //locate the weld pool

		if (data.n_contours == 0) { //signal that the welder is turned off
			//putText(original, "Welder is Off", Point(200, 100), FONT_HERSHEY_COMPLEX, 0.6, Scalar(255, 0, 255), 1);
			welderOffFrames.push_back(data.n_frame);
		}
		
		if (data.n_frame != 0) {
			if (dropped(previous, original, data)) {
				skippedFrames.push_back(data.n_frame);
			}

			if (isStill(previous, original, data)) {
				cout << "bitches\n";
				for (int i = 0; i < 10; ++i) {
					stillFrames.push_back(data.n_frame - i);
				}
				data.stillCount = 0;
			}
		}

		//display the image(s)
		//imshow("thresholded", data.frame);
		imshow("original", original);
		//imshow("gray", data.gray);

		previous = original.clone(); //store the previous frame
		data.n_frame++; 
		//waitKey(0);
		char c = waitKey(1);
		if (c == 27) //press esc to	terminate video processing early
			break;
	}

	//write results to file
	for (int i = 0; i < stillFrames.size(); ++i) 
		output << "still motion at frame " << stillFrames[i] << endl;
	output << endl;

	for (int i = 0; i < welderOffFrames.size(); ++i)
		output << "welder off at frame " << welderOffFrames[i] << endl;
	output << endl;

	for (int i = 0; i < skippedFrames.size(); ++i) {
		output << "skipped frames at frame " << skippedFrames[i] << endl;
	}
	output << endl;

	for (int i = 0; i < welderAngles.size(); ++i) {
		output << "weld path inclination at frame " << i + 1 << ": " << welderAngles[i] << endl;
	}
 
}

//wrapper
void Video::toGray(Frame& data) {
	cvtColor(data.frame, data.frame, CV_BGR2GRAY);
	data.gray = data.frame.clone();
}

//wrapper
void Video::resize(Mat& frame, double factor) {
	cv::resize(frame, frame, Size(), factor, factor);
}

//wrapper
void Video::threshold(Mat& frame, int low) {
	cv::threshold(frame, frame, low, 255, THRESH_BINARY);
	dilate(frame, frame, Mat(), Point(-1, -1), 1);
}

//detects the weld pool
//precondition: binary image is used to detect contours
//postcondition: weld pool has been located in the original frame
void Video::drawHull(Frame& data, Mat& original, vector<int>& skippedFrames) {
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(data.frame, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
	vector<vector<Point> > hull(contours.size());
	int largestIndex = -1;
	int largest = -1;
	int secondLargestIndex = -1;

	//following code filters out the invalid contours and finds the weld pool contour
	int validContoursCount = 0;
	for (int i = 0; i < contours.size(); ++i) {
		double area = contourArea(contours[i]); //calculate the area...
		double perimeter = arcLength(contours[i], true); //...and perimeter of the contour detected
		if (area > 1700 && (perimeter > 303 || area > largest) && (perimeter > 303 || area/perimeter < 15.7)) { //filter out the false positive contours by going through a series of tests
			validContoursCount++;
			
			//we need to find the second largest contour in the frame in case we grab the wrong contour, we can use the center of mass check to find the correct contour
			if (area > largest) {
				data.secondLargestContourArea = largest;
				secondLargestIndex = largestIndex;
				largest = area;
				largestIndex = i;
			}
			else if (area > data.secondLargestContourArea && area != largest) { 
				data.secondLargestContourArea = area;
				secondLargestIndex = i;
			}
		}
	}
	data.n_contours = validContoursCount;

	if (largestIndex != -1) { //if a valid contour has been found
		bool movedTooMuch = false; //flag for determining whether or not the center of mass of the contour has moved too much between the current frame and the previous frame
		data.oldContourArea = data.newContourArea; //update the contour area of the previous frame and current frame
		data.newContourArea = largest;	
		
		Moments moment = moments(contours[largestIndex]); //calculate the center of mass of the largest contour
		data.currentCM = Point2f(moment.m10/moment.m00, moment.m01/moment.m00); 

		if ((abs(data.currentCM.x - data.oldCM.x) > MAX_CENTER_DIFF_X || abs(data.currentCM.y - data.oldCM.y) > MAX_CENTER_DIFF_Y) && data.oldCM.x != 0 && data.oldCM.y != 0)  //center has moved too much
			movedTooMuch = true;

		if (movedTooMuch && validContoursCount > 1) { //if there is more than on valid contour and the contour has moved too much, chances are that we chose the wrong contour...in that case, the correct
			largestIndex = secondLargestIndex;		  //contour should be the second largest one
			moment = moments(contours[largestIndex]);
			data.currentCM = Point2f(moment.m10 / moment.m00, moment.m01 / moment.m00);
		}
		else if (movedTooMuch)
			skippedFrames.push_back(data.n_frame);
		
		//draw the convex hull around the weld pool on the original image
		convexHull(Mat(contours[largestIndex]), hull[largestIndex], false);
		drawContours(original, hull, largestIndex, Scalar(255, 0, 0), 3);
		data.oldCM = data.currentCM;
	}
	cout << endl;
}

//assuming that a dropped frame means one that hasn't changed from the previous one
//determines which frames were dropped/skipped by checking the difference in pixel intensity between the previous frame and the current frame
//we have to be pretty strict about determining which frames are dropped since the difference in pixel intensity should be pretty close to 0
bool Video::dropped(const Mat& previous, const Mat& current, Frame& frame) {
	if (abs(frame.oldContourArea - frame.newContourArea) > MAX_CONTOUR_AREA_DIFF)
		return false;

	int unstablePixelCount = 0;

	for (int i = previous.rows / 4; i < previous.rows; ++i) { //we don't have to scan the entire frame because the only parts that change intensity a lot are the areas close to the weld pool
		for (int j = previous.cols / 4; j < previous.cols; ++j) { //this saves time
			if (abs(previous.at<uchar>(i, j) - current.at<uchar>(i, j)) > MAX_PIXEL_DIFF_DROPPING) //calculate the difference in intensity of pixels in the previous frame and the current frame
				unstablePixelCount++; 

			//if the number of "different" pixels is over the threshold, we assume that the frame was dropped
			if (unstablePixelCount >= MAX_UNSTABLE_CONTOURS_DROPPING) 
				return false;
		}
	}
	return true;

}

//precondition: valid grayscale frames are passed in
//postcondition: determines whether or not there was motion of the welder in between several frames
//stillCount counts the number of consecutive frames in which there is little to no motion
int Video::isStill(const Mat& previous, const Mat& current, Frame& frame) {
	if (abs(frame.oldContourArea - frame.newContourArea) > MAX_CONTOUR_AREA_DIFF) {

		frame.stillCount = 0;
		return false;
	}

	int unstablePixelCount = 0;
	for (int i = previous.rows/4; i < previous.rows; i += 3) { //optimization
		for (int j = previous.cols/4; j < previous.cols; j += 3) {
			if (abs(previous.at<uchar>(i, j) - current.at<uchar>(i, j)) > MAX_PIXEL_DIFF_STILL) //calculate the difference in intensity of pixels in the previous frame and the current frame
				unstablePixelCount++;

			//if the number of "different" pixels is over the limit, we assume that there was motion
			if (unstablePixelCount >= MAX_UNSTABLE_CONTOURS_STILL) {
				frame.stillCount = 0; //set the counter back to 0
				return false;
			}
		}
	}

	//if code can reach this point, we know that there was little to no motion in between the 2 frames...therefore we can increase stillCount
	frame.stillCount++;
	if (frame.stillCount >= 10) { //assume that if there are 10 consecutive frames in which the pixel intensities don't vary much, assume that there was motion stopping
		cout << "frame count suckas\n";
		return true;
	}

	return false;
}

//postcondition: valid grayscale image is passed in
//postcondition: calculates the inclination of the weld path
void Video::calcWelderAngle(Frame& frame) {
	Canny(frame.gray, frame.gray, 20, 70); //detect edges in the grayscale frame
	vector<Vec2f> lines;
  	HoughLines(frame.gray, lines, 1, CV_PI / 180, 300); //find the red line in the frame
	double length = -1; //length of the longest line found

	//trig to calculate the length of the longest line found
	for (int i = 0; i < lines.size(); i++)
	{
		double rho = lines[i][0];
		double theta = lines[i][1];
		double xComp = cos(theta), yComp = sin(theta);
		double x = xComp * rho, y = yComp * rho;
		Point pt1((x + 10000000 * (-yComp)), (y + 10000000 * (xComp)));
		Point pt2((x - 10000000 * (-yComp)), (y - 10000000 * (xComp)));

		double slope = static_cast<double>(pt2.y - pt1.y) / (pt2.x - pt1.x); //calculate slope of the line...
		pt1.y = slope * (0 - pt1.x) + pt1.y; //and find the values of y when x is at the edge of the frame ie x = 0 and x = width of frame
		pt1.x = 0;
		pt2.y = slope * (frame.gray.cols - pt2.x) + pt2.y;
		pt2.x = frame.gray.cols;

		if (norm(pt2 - pt1) > length) //find the longest line in the frame
			length = norm(pt2 - pt1);
	}

	//if the red line is not found, assume that the inclination is 0 degrees...otherwise, calculate the angle of the inclination	 /
	//a = length of red line																									  x	/
    //b = width of image																										   /_______
	//

		
	if (length == -1)
		frame.welderAngle = 0;
	else
		frame.welderAngle = acos(frame.gray.cols / length) * 180 / PI;
}

