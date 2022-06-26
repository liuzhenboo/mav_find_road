#pragma once
#include "mypoint.h"
#include <vector>
#include <list>
#include <map>
#include <opencv/cv.h>
#include <fstream>
#include <iostream>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;
class mypoint;

class cell
{
public:
	cell();
	cell(unsigned int x_index, unsigned int y_index);
	~cell();
	// 语义类别
	std::string lable_;
	// 平均深度
	double ave_depth_;
	double ave_x_, ave_y_, ave_z_;

	unsigned int x_index_;
	unsigned int y_index_;

	vector<double> normal_;

	std::vector<mypoint *> image_ponts_;

	// "obs","ground","no_occupied", had_dilate.
	std::string status_;
};
