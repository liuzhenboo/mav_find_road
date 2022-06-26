#pragma once
#include "mypoint.h"

#include <vector>
#include <list>
#include <map>
// #include <multimap>
#include <opencv/cv.h>
#include <fstream>
#include <iostream>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;
using namespace std;
class mypoint;
class ring_cell
{
public:
	ring_cell();
	ring_cell(unsigned int index_x, unsigned int index_y, string status);

	~ring_cell();
	unsigned int index_x_, index_y_;
	std::multimap<float, mypoint *> ring_ponts_;
	//: all_obs, all_ground, obs_and_ground ,just_init,
	string status_;
};
