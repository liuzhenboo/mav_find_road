
#include <Map.h>
#include <memory>
#include <iostream>
Map::Map()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp1(new pcl::PointCloud<pcl::PointXYZRGB>);

	all_pc = temp;
	all_pc_obs = temp1;
	length = 30.0;
	width = 30.0;
	resolution = 0.2;
}
Map::~Map()
{
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr Map::get_allmap()
{
	return all_pc;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Map::get_allObsmap()
{
	return all_pc_obs;
}
bool Map::fusion(pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_pt)
{
	*all_pc = *all_pc + *current_pt;
	return true;
}

bool Map::fusion_obs(pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_pt)
{
	*all_pc_obs = *all_pc_obs + *current_pt;
	return true;
}

bool Map::fusion_cell(pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_pts)
{
	std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB>> data = current_pts->points;
	for (unsigned int i = 0; i < data.size(); i++)
	{
		addOnePoint(data[i].x, data[i].y, data[i].z);
	}
	return true;
}

// 投射到格子iｄ号，更新状态量
void Map::addOnePoint(float x, float y, float z)
{
	int current_id = Corrd2Id(x, y, z);
	// TODO get current_id...
	std::cout << "ADD one point" << std::endl;
	if (current_id < sum_size)
	{ // 如果已经存在，那就融合高度；否则创建新的
		std::cout << "current_id: " << current_id << std::endl;
		std::cout << "sum_size: " << sum_size << std::endl;

		if (road_ids.find(current_id) != road_ids.end())
		{
			std::cout << "ADD one point1" << std::endl;

			cellDataset_[current_id]->Update(1, z);
		}
		else
		{
			std::cout << "ADD one point2" << std::endl;

			cellDataset_[current_id] = new Cell(1, z);
			road_ids.insert(current_id);
		}
	}
	else
	{
		std::cout << "超出地图！" << std::endl;
	}
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Map::get_cellMap()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_vis(new pcl::PointCloud<pcl::PointXYZ>);
	std::set<int>::iterator iter = road_ids.begin();
	while (iter != road_ids.end())
	{
		/* code */
		std::vector<float> current_ground_point = Id2Corrd(*iter);
		pcl::PointXYZ pt;
		pt.x = current_ground_point[0];
		pt.y = current_ground_point[1];
		pt.z = current_ground_point[2];

		cloud_vis->push_back(pt);
		iter++;
	}
	return cloud_vis;
}

std::vector<float> Map::Id2Corrd(int id)
{
	std::vector<float> reasult;
	reasult.push_back(((id + 1) / length_size - origin_id_x) * resolution);
	reasult.push_back(((id + 1) % length_size - origin_id_y) * resolution);
	reasult.push_back(cellDataset_[id]->GetZ(id));
	return reasult;
}
int Map::Corrd2Id(float x, float y, float z)
{
	int ans = static_cast<int>((x - rd_x) / resolution + 1) * length_size + static_cast<int>((y - rd_y) / resolution);
	return ans;
}
void Map::init(ros::NodeHandle &nh)
{
	length_size = length / resolution;
	width_size = width / resolution;
	sum_size = length_size * width_size;
	cellDataset_.resize(sum_size);
	origin_id = static_cast<int>(width_size / 2) * length_size + static_cast<int>(length_size / 2) - 1;

	// 以右下角为原点(1,1)，便于id2cord...，
	origin_id_x = static_cast<int>(width_size / 2) + 1;
	origin_id_y = static_cast<int>(length_size / 2);

	rd_x = -1.0 * origin_id_x * resolution;
	rd_y = -1.0 * origin_id_y * resolution;
}