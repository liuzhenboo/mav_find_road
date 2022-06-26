#pragma once
#include <vector>
#include <list>
#include <map>
#include <opencv/cv.h>
#include <fstream>
#include <iostream>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "mypoint.h"
#include <time.h>
//PCL
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

// PCL 库
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

//YAML
#include <iostream>
#include "yaml-cpp/yaml.h"
#include <fstream>

#define debug_point_to_ringmap_project
#define debug_point_to_imgmap_project

using namespace cv;
using namespace std;
// class mypoint;
class mysystem
{
private:
	// 外部图像数据
	//均为像素坐标系
	float camera_cx_,
		camera_cy_,
		camera_fx_,
		camera_fy_,
		camera_f_;
	float camera_factor_;
	cv::Mat depth_img_;
	cv::Mat rgb_img_;
	cv::Mat true_img_;

	//图像格子数据库
	// 通过这四个map来管理格子
	// 其中通过edge_cells_来间接管理另外四个
	int col_;
	int row_;
	int col_percell_;
	int row_percell_;
	int col_cellsum_;
	int row_cellsum_;
	int pix_sum_percell_;
	vector<vector<cell>> cellmap_;
	vector<vector<cell>> true_cellmap_;
	vector<cell *> edge_map_;
	map<string, cell *> ground_cells_;
	map<string, cell *> obs_cells_;
	map<string, cell *> unsure_cells_;
	map<string, cell *> edge_cells_;
	map<string, cell *> unsearch_cells_;
	vector<double> weight_;
	int edge_dilate_size_;
	int kenel_size_;

	//环形格子数据库
	float ring_r_;
	float ring_R_;
	float ring_theta_;
	int ring_row_;
	int ring_col_;
	vector<vector<ring_cell *>> ring_cellmap_;
	float min_obs_;
	map<string, ring_cell *> unexplored_ring_cells_;
	map<string, ring_cell *> explored_ring_cells;
	map<string, ring_cell *> edge_ring_cells;
	float max_between_cell_high_;
	float voxfile_;

	//点云数据库，将y值作为键值，用于排序
	std::multimap<float, mypoint *>
		ponts_db_;
	// function
public:
	void resize_cellmap(unsigned int col, unsigned int row, unsigned int col_percell, unsigned int row_percell);
	void getdata(cv::Mat idepth, cv::Mat itrue);
	void run();

private:
	void evaluate();
	void update_true_cell(unsigned int i_index, unsigned int j_index);
	void get_pcl_pointcloud();
	void debug_mypoint(string name);
	void debug_mypoint_true_labelme();
	void segment_based_height();
	void segment_based_wave();
	void update_image_map_lable();
	void visual_image_map_lable();
	void visual_image_map_true_lable();

	void optimize_segment();
	void get_dilate_edge();
	bool detect_dilate(unsigned int i, unsigned int j);
	void update_mypoints_status(ring_cell *last, ring_cell *current);
	void get_true_lable_image_map();
	void calculate_normal();
};