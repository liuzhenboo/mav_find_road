// ./depth_extract /home/lzb/Ground_Pointcloud_Extract/src/depth_extract/data/rgb/rgb_out_json/label.png  /home/lzb/Ground_Pointcloud_Extract/src/depth_extract/data/depth/depth_out.png /home/lzb/Ground_Pointcloud_Extract/src/depth_extract/data/rgb/rgb_out_json/label.png
#include <vector>
#include <list>
#include <opencv/cv.h>
#include <fstream>
#include <iostream>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "mysystem.h"
using namespace cv;

int main(int argc, char **argv)
{
	cv::Mat input_rgb;
	cv::Mat input_depth;
	// 建议640, 480
	// 输入彩色图
	input_rgb = cv::imread(argv[1], CV_LOAD_IMAGE_UNCHANGED);
	// 输入深度图
	input_depth = cv::imread(argv[2], CV_LOAD_IMAGE_UNCHANGED);

	// 为手动标记的彩色图构造网格图
	unsigned int col_rgb = input_rgb.cols;
	unsigned int row_rgb = input_rgb.rows;

	unsigned int col_depth = input_depth.cols;
	unsigned int row_depth = input_depth.rows;

	if (col_rgb != col_depth || row_rgb != row_depth)
	{
		std::cout << "深度图与彩色图维度不一致" << std::endl;
	}
	else
	{
		std::cout << "col  and row is: ";
		std::cout << col_rgb << ", " << row_rgb << std::endl;
		std::cout << "彩色图通道数：" << input_rgb.channels() << std::endl;
		std::cout << "深度图通道数：" << input_depth.channels() << std::endl;
	}

	//---------------------------------------------------------------------------
	mysystem image_map_rgb;
	mysystem image_map_depth;

	// 每个格子的像素数目
	cv::FileStorage fsSettings("../src/config.yaml", cv::FileStorage::READ);
	if (!fsSettings.isOpened())
	{
		cerr << "ERROR: Wrong config path at : " << endl;
		exit(-1);
	}
	int c = fsSettings["col_percell"];
	int r = fsSettings["row_percell"];

	unsigned int col_percell = c;
	unsigned int row_percell = r;

	image_map_rgb.getdata(input_depth, input_rgb);
	image_map_rgb.resize_cellmap(col_rgb, row_rgb, col_percell, row_percell);

	image_map_rgb.run();
	// show-----------------------------------------------------
	// Point p1(100, 100);
	// Point p2(758, 50);
	// line(input_rgb, p1, p2, Scalar(33, 33, 133), 2);
	// imshow("画矩形", input_rgb);
	// waitKey();

	// cv::Mat out_rgb, out_depth;

	// namedWindow("rgb", WINDOW_AUTOSIZE);
	// imshow("rgb", input_rgb);
	// waitKey(0);
	// namedWindow("depth", WINDOW_AUTOSIZE);
	// imshow("depth", input_depth);
	// waitKey(0);

	//imwrite("orb_features.jpg", out_rgb);
	return 0;
}