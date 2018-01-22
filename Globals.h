#ifndef GLOBALS_H
#define GLOBALS_H


//global constants
const int THRESH_VALUE = 134;
const int MAX_PIXEL_DIFF_STILL = 8; //max pixel intensity difference between 2 frames to be considered still
const int MAX_PIXEL_DIFF_DROPPING = 5;
const int MAX_CENTER_DIFF_X = 80; //max center of contours horizontal difference before frames are considered to skip/drop
const int MAX_CENTER_DIFF_Y = 50; //max center of contours vertical difference before frames are considered to skip/drop
const int MAX_CONTOUR_AREA_DIFF = 220; //max contour area difference between 2 frames to be considered still
const int MAX_UNSTABLE_CONTOURS_STILL = 140; //max number of different pixels in two frames before the frames are not still
const int MAX_UNSTABLE_CONTOURS_DROPPING = 30;
const double PI = 3.14159265359;

#endif