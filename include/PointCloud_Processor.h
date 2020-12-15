

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/console.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>

#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud2.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_mapping.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/Compression.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/ULogger.h>
#include <pcl_conversions/pcl_conversions.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <rtabmap/core/util3d_surface.h>
#include <geometry_msgs/Transform.h>

#include "rtabmap/core/OccupancyGrid.h"
#include "rtabmap/utilite/UStl.h"
#include <boost/mpl/bind.hpp>
#include <iostream>
using namespace rtabmap;
class PointCloud_Processor
{
public:
	PointCloud_Processor(/* args */);
	~PointCloud_Processor();
	pcl::PointCloud<pcl::PointXYZ>::Ptr segmentCloud(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudIn,
		const pcl::IndicesPtr &indicesIn,
		const Transform &pose,
		const cv::Point3f &viewPoint,
		pcl::IndicesPtr &groundIndices,
		pcl::IndicesPtr &obstaclesIndices,
		pcl::IndicesPtr *flatObstacles);

	void segmentObstaclesFromGround(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
		const pcl::IndicesPtr &indices,
		pcl::IndicesPtr &ground,
		pcl::IndicesPtr &obstacles,
		int normalKSearch,
		float groundNormalAngle,
		float clusterRadius,
		int minClusterSize,
		bool segmentFlatObstacles,
		float maxGroundHeight,
		pcl::IndicesPtr *flatObstacles,
		const Eigen::Vector4f &viewPoint);

	void pc_init();
	void double2float();
	bool preVoxelFiltering_ = true;
	bool projMapFrame_ = false;

	bool normalsSegmentation_ = true;
	bool groundIsObstacle_ = false;
	bool flatObstaclesDetected_ = true;
	int minClusterSize_ = 4;
	int noiseFilteringMinNeighbors_ = 5;
	int normalKSearch_ = 10;

	//float
	float cellSize_ = 0.04;
	float footprintLength_ = 0.0;
	float footprintWidth_ = 0.0;
	float footprintHeight_ = 0.0;
	float minGroundHeight_ = 0.0;
	float maxObstacleHeight_ = 2.0;
	float maxGroundAngle_ = 0.785;
	float clusterRadius_ = 0.1;
	float noiseFilteringRadius_;
	float maxGroundHeight_ = 0.0;

	// double
	double cellSize = 0.04;
	double footprintLength = 0.0;
	double footprintWidth = 0.0;
	double footprintHeight = 0.0;
	double minGroundHeight = 0.0;
	double maxObstacleHeight = 2.0;
	double maxGroundAngle = 0.785;
	double clusterRadius = 0.1;
	double noiseFilteringRadius;
	double maxGroundHeight = 0.0;
};