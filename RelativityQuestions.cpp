// RelativityQuestions.cpp : Defines the entry point for the console application.

#include "stdafx.h"
#include "Video.h"
#include <iostream>

using namespace std;
using namespace cv;

int main()
{

	Video video("weld_video.mp4");
	video.display();
	return 0;
}

