#pragma once
#include "image_cell.h"
#include "ring_cell.h"
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

class cell;
class ring_cell;
class mypoint
{
public:
	mypoint();

	~mypoint();

	//需要指向网格图
	cell *cell_;
	// 需要指向环形地图
	ring_cell *ring_cell_;
	float x_, y_, z_, r_, g_, b_;
	// ground =11;obs=22; init=33
	float depth_;
	unsigned int is_ground_;
};
