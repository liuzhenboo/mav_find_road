#include <iostream>
#include <ros/ros.h>
#include <set>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/io.h>
#include <Cell.h>
class Map
{
public:
	Map();
	~Map();
	void init(ros::NodeHandle &nh);

	// 点云拼接融合
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_allmap();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_allObsmap();
	bool fusion(pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_pts);
	bool fusion_obs(pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_pts);
	// 方格融合
	bool fusion_cell(pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_pts);

	void addOnePoint(float x, float y, float z);
	pcl::PointCloud<pcl::PointXYZ>::Ptr get_cellMap();
	int Corrd2Id(float x, float y, float z);
	std::vector<float> Id2Corrd(int id);

private:
	// 点云　fusion
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr all_pc;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr all_pc_obs;
	// 方格fusion
	std::vector<Cell *> cellDataset_;
	std::set<int> road_ids;
	std::set<int> obs_ids;

	double length = 30;
	double width = 30;
	double resolution = 0.2;
	int length_size;
	int width_size;
	int sum_size;
	int origin_id;
	int origin_id_x;
	int origin_id_y;
	double rd_x;
	double rd_y;
};