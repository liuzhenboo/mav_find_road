
#include <rtabmap/core/util3d_mapping.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/utilite/ULogger.h>

#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/util3d.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/io.h>
#include <PointCloud_Processor.h>
using namespace util3d;
PointCloud_Processor::PointCloud_Processor(/* args */)
{
}

PointCloud_Processor::~PointCloud_Processor()
{
}
void PointCloud_Processor::pc_init()
{
	preVoxelFiltering_ = true;
	cellSize_ = 0.04;
	projMapFrame_ = false;
	footprintLength_ = 0.0;
	footprintWidth_ = 0.0;
	footprintHeight_ = 0.0;
	minGroundHeight_ = 0.0;
	maxObstacleHeight_ = 2.0;
	normalsSegmentation_ = true;
	groundIsObstacle_ = false;
	maxGroundAngle_ = 0.785;
	clusterRadius_ = 0.1;
	flatObstaclesDetected_ = true;
	minClusterSize_ = 4;
	noiseFilteringRadius_;
	noiseFilteringMinNeighbors_ = 5;
	normalKSearch_ = 10;
	maxGroundHeight_ = 0.0;
}
void PointCloud_Processor::double2float()
{
	cellSize_ = cellSize;
	footprintLength_ = footprintLength;
	footprintWidth_ = footprintWidth;
	footprintHeight_ = footprintHeight;
	minGroundHeight_ = minGroundHeight;
	maxObstacleHeight_ = maxObstacleHeight;
	maxGroundAngle_ = maxGroundAngle;
	clusterRadius_ = clusterRadius;
	noiseFilteringRadius_ = noiseFilteringRadius;
	maxGroundHeight_ = maxGroundHeight;
}

// void PointCloud_Processor::segmentObstaclesFromGround(
// 	const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
// 	const pcl::IndicesPtr &indices,
// 	pcl::IndicesPtr &ground,
// 	pcl::IndicesPtr &obstacles,
// 	int normalKSearch,
// 	float groundNormalAngle,
// 	float clusterRadius,
// 	int minClusterSize,
// 	bool segmentFlatObstacles,
// 	float maxGroundHeight,
// 	pcl::IndicesPtr *flatObstacles,
// 	const Eigen::Vector4f &viewPoint)
// {
// 	ground.reset(new std::vector<int>);
// 	obstacles.reset(new std::vector<int>);
// 	if (flatObstacles)
// 	{
// 		flatObstacles->reset(new std::vector<int>);
// 	}

// 	if (cloud->size())
// 	{
// 		// (1)提取所有平面
// 		// pcl::NormalEstimationOMP<PointT, pcl::Normal>,使用OpenMP标准并行估计每个3D点的局部表面属性;
// 		//normalKSearch为求法线时候需要的拟合点
// 		pcl::IndicesPtr flatSurfaces = util3d::normalFiltering(
// 			cloud,
// 			indices,
// 			groundNormalAngle,
// 			Eigen::Vector4f(0, 0, 1, 0),
// 			normalKSearch,
// 			viewPoint);

// 		//(2)聚类,得到最大的平面,以及满足路高低要求的平面.
// 		// 去除flatSurfaces中的障碍物,得到ground点云
// 		if (segmentFlatObstacles && flatSurfaces->size())
// 		{
// 			int biggestFlatSurfaceIndex;
// 			//pcl::EuclideanClusterExtraction分类
// 			// minClusterSize 聚类最小半径
// 			// std::numeric_limits<int>::max() 聚类最大半径
// 			std::vector<pcl::IndicesPtr> clusteredFlatSurfaces = util3d::extractClusters(
// 				cloud,
// 				flatSurfaces,
// 				clusterRadius,
// 				minClusterSize,
// 				std::numeric_limits<int>::max(),
// 				&biggestFlatSurfaceIndex);

// 			// cluster all surfaces for which the centroid is in the Z-range of the bigger surface
// 			if (clusteredFlatSurfaces.size())
// 			{
// 				ground = clusteredFlatSurfaces.at(biggestFlatSurfaceIndex);
// 				// min和max为所有点中坐标最小点与最大点
// 				Eigen::Vector4f min, max;
// 				pcl::getMinMax3D(*cloud, *clusteredFlatSurfaces.at(biggestFlatSurfaceIndex), min, max);

// 				// 若平面点云的最低点小于指定的道路最大高度;否则则不存在这样的道路
// 				if (maxGroundHeight == 0.0f || min[2] < maxGroundHeight)
// 				{
// 					for (unsigned int i = 0; i < clusteredFlatSurfaces.size(); ++i)
// 					{
// 						if ((int)i != biggestFlatSurfaceIndex)
// 						{
// 							// 计算质心centroid, 根据质心判断除了最大平面,其他平面应该放在ground中,还是放在flatObstacles中.
// 							Eigen::Vector4f centroid(0, 0, 0, 1);
// 							pcl::compute3DCentroid(*cloud, *clusteredFlatSurfaces.at(i), centroid);
// 							if (maxGroundHeight == 0.0f || centroid[2] <= maxGroundHeight || centroid[2] <= max[2]) // epsilon
// 							{
// 								ground = util3d::concatenate(ground, clusteredFlatSurfaces.at(i));
// 							}
// 							else if (flatObstacles)
// 							{
// 								*flatObstacles = util3d::concatenate(*flatObstacles, clusteredFlatSurfaces.at(i));
// 							}
// 						}
// 					}

// 					// (3) 提取内点,得到道路
// 					// 再次检测,提取路的内点
// 					pcl::ModelCoefficients coefficients;
// 					std::cout << "平面检测" << std::endl;
// 					ground = util3d::extractPlane(cloud, ground, 0.02, 100, &coefficients);
// 				}
// 				else
// 				{
// 					// reject ground!
// 					ground.reset(new std::vector<int>);
// 					if (flatObstacles)
// 					{
// 						*flatObstacles = flatSurfaces;
// 					}
// 				}
// 			}
// 		}
// 		else
// 		{
// 			std::cout << "不对平面进行障碍物剔除!" << std::endl;
// 			ground = flatSurfaces;
// 		}

// 		if (ground->size() != cloud->size())
// 		{
// 			// Remove ground
// 			pcl::IndicesPtr notObstacles = ground;
// 			if (indices->size())
// 			{
// 				notObstacles = util3d::extractIndices(cloud, indices, true);
// 				notObstacles = util3d::concatenate(notObstacles, ground);
// 			}
// 			pcl::IndicesPtr otherStuffIndices = util3d::extractIndices(cloud, notObstacles, true);

// 			// If ground height is set, remove obstacles under it
// 			if (maxGroundHeight != 0.0f)
// 			{
// 				otherStuffIndices = rtabmap::util3d::passThrough(cloud, otherStuffIndices, "z", maxGroundHeight, std::numeric_limits<float>::max());
// 			}

// 			//Cluster remaining stuff (obstacles)
// 			if (otherStuffIndices->size())
// 			{
// 				std::vector<pcl::IndicesPtr> clusteredObstaclesSurfaces = util3d::extractClusters(
// 					cloud,
// 					otherStuffIndices,
// 					clusterRadius,
// 					minClusterSize);

// 				// merge indices
// 				obstacles = util3d::concatenate(clusteredObstaclesSurfaces);
// 			}
// 		}
// 	}
// }

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud_Processor::segmentCloud(
	const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudIn,
	const pcl::IndicesPtr &indicesIn,
	const Transform &pose,
	const cv::Point3f &viewPoint,
	pcl::IndicesPtr &groundIndices,
	pcl::IndicesPtr &obstaclesIndices,
	pcl::IndicesPtr *flatObstacles)
{
	groundIndices.reset(new std::vector<int>);
	obstaclesIndices.reset(new std::vector<int>);
	if (flatObstacles)
	{
		flatObstacles->reset(new std::vector<int>);
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::IndicesPtr indices(new std::vector<int>);

	if (preVoxelFiltering_)
	{
		// voxelize to grid cell size
		cloud = util3d::voxelize(cloudIn, indicesIn, cellSize_);

		indices->resize(cloud->size());
		for (unsigned int i = 0; i < indices->size(); ++i)
		{
			indices->at(i) = i;
		}
	}
	else
	{
		cloud = cloudIn;
		if (indicesIn->empty() && cloud->is_dense)
		{
			indices->resize(cloud->size());
			for (unsigned int i = 0; i < indices->size(); ++i)
			{
				indices->at(i) = i;
			}
		}
		else
		{
			indices = indicesIn;
		}
	}

	// add pose rotation without yaw
	float roll, pitch, yaw;
	pose.getEulerAngles(roll, pitch, yaw);
	UDEBUG("node.getPose()=%s projMapFrame_=%d", pose.prettyPrint().c_str(), projMapFrame_ ? 1 : 0);
	cloud = util3d::transformPointCloud(cloud, Transform(0, 0, projMapFrame_ ? pose.z() : 0, roll, pitch, 0));

	// filter footprint
	if (footprintLength_ > 0.0f || footprintWidth_ > 0.0f || footprintHeight_ > 0.0f)
	{
		indices = util3d::cropBox(
			cloud,
			indices,
			Eigen::Vector4f(
				footprintLength_ > 0.0f ? -footprintLength_ / 2.0f : std::numeric_limits<int>::min(),
				footprintWidth_ > 0.0f && footprintLength_ > 0.0f ? -footprintWidth_ / 2.0f : std::numeric_limits<int>::min(),
				0,
				1),
			Eigen::Vector4f(
				footprintLength_ > 0.0f ? footprintLength_ / 2.0f : std::numeric_limits<int>::max(),
				footprintWidth_ > 0.0f && footprintLength_ > 0.0f ? footprintWidth_ / 2.0f : std::numeric_limits<int>::max(),
				footprintHeight_ > 0.0f && footprintLength_ > 0.0f && footprintWidth_ > 0.0f ? footprintHeight_ : std::numeric_limits<int>::max(),
				1),
			Transform::getIdentity(),
			true);
	}

	/// 取得一定范围内点云的索引:indices
	// filter ground/obstacles zone
	if (minGroundHeight_ != 0.0f || maxObstacleHeight_ != 0.0f)
	{
		indices = util3d::passThrough(cloud, indices, "z",
									  minGroundHeight_ != 0.0f ? minGroundHeight_ : std::numeric_limits<int>::min(),
									  maxObstacleHeight_ > 0.0f ? maxObstacleHeight_ : std::numeric_limits<int>::max());
		UDEBUG("indices after max obstacles height filtering = %d", (int)indices->size());
	}

	if (indices->size())
	{
		if (normalsSegmentation_ && !groundIsObstacle_)
		{
			UDEBUG("normalKSearch=%d", normalKSearch_);
			UDEBUG("maxGroundAngle=%f", maxGroundAngle_);
			UDEBUG("Cluster radius=%f", clusterRadius_);
			UDEBUG("flatObstaclesDetected=%d", flatObstaclesDetected_ ? 1 : 0);
			UDEBUG("maxGroundHeight=%f", maxGroundHeight_);
			segmentObstaclesFromGround(
				cloud,
				indices,
				groundIndices,
				obstaclesIndices,
				normalKSearch_,
				maxGroundAngle_,
				clusterRadius_,
				minClusterSize_,
				flatObstaclesDetected_,
				maxGroundHeight_,
				flatObstacles,
				Eigen::Vector4f(viewPoint.x, viewPoint.y, viewPoint.z + (projMapFrame_ ? pose.z() : 0), 1));
			UDEBUG("viewPoint=%f,%f,%f", viewPoint.x, viewPoint.y, viewPoint.z + (projMapFrame_ ? pose.z() : 0));
			//UWARN("Saving ground.pcd and obstacles.pcd");
			//pcl::io::savePCDFile("ground.pcd", *cloud, *groundIndices);
			//pcl::io::savePCDFile("obstacles.pcd", *cloud, *obstaclesIndices);
		}
		else
		{
			UDEBUG("");
			// passthrough filter
			groundIndices = rtabmap::util3d::passThrough(cloud, indices, "z",
														 minGroundHeight_ != 0.0f ? minGroundHeight_ : std::numeric_limits<int>::min(),
														 maxGroundHeight_ != 0.0f ? maxGroundHeight_ : std::numeric_limits<int>::max());

			pcl::IndicesPtr notObstacles = groundIndices;
			if (indices->size())
			{
				notObstacles = util3d::extractIndices(cloud, indices, true);
				notObstacles = util3d::concatenate(notObstacles, groundIndices);
			}
			obstaclesIndices = rtabmap::util3d::extractIndices(cloud, notObstacles, true);
		}

		UDEBUG("groundIndices=%d obstaclesIndices=%d", (int)groundIndices->size(), (int)obstaclesIndices->size());

		// Do radius filtering after voxel filtering ( a lot faster)
		if (noiseFilteringRadius_ > 0.0 && noiseFilteringMinNeighbors_ > 0)
		{
			UDEBUG("");
			if (groundIndices->size())
			{
				groundIndices = rtabmap::util3d::radiusFiltering(cloud, groundIndices, noiseFilteringRadius_, noiseFilteringMinNeighbors_);
			}
			if (obstaclesIndices->size())
			{
				obstaclesIndices = rtabmap::util3d::radiusFiltering(cloud, obstaclesIndices, noiseFilteringRadius_, noiseFilteringMinNeighbors_);
			}
			if (flatObstacles && (*flatObstacles)->size())
			{
				*flatObstacles = rtabmap::util3d::radiusFiltering(cloud, *flatObstacles, noiseFilteringRadius_, noiseFilteringMinNeighbors_);
			}

			if (groundIndices->empty() && obstaclesIndices->empty())
			{
				UWARN("Cloud (with %d points) is empty after noise "
					  "filtering. Occupancy grid cannot be "
					  "created.",
					  (int)cloud->size());
			}
		}
	}
	return cloud;
}
void PointCloud_Processor::segmentObstaclesFromGround(
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
	const Eigen::Vector4f &viewPoint)
{
	ground.reset(new std::vector<int>);
	obstacles.reset(new std::vector<int>);
	if (flatObstacles)
	{
		flatObstacles->reset(new std::vector<int>);
	}

	if (cloud->size())
	{
		// Find the ground
		pcl::IndicesPtr flatSurfaces = normalFiltering(
			cloud,
			indices,
			groundNormalAngle,
			Eigen::Vector4f(0, 0, 1, 0),
			normalKSearch,
			viewPoint);

		if (segmentFlatObstacles && flatSurfaces->size())
		{
			int biggestFlatSurfaceIndex;
			std::vector<pcl::IndicesPtr> clusteredFlatSurfaces = util3d::extractClusters(
				cloud,
				flatSurfaces,
				clusterRadius,
				minClusterSize,
				std::numeric_limits<int>::max(),
				&biggestFlatSurfaceIndex);

			// cluster all surfaces for which the centroid is in the Z-range of the bigger surface
			if (clusteredFlatSurfaces.size())
			{
				ground = clusteredFlatSurfaces.at(biggestFlatSurfaceIndex);
				Eigen::Vector4f min, max;
				pcl::getMinMax3D(*cloud, *clusteredFlatSurfaces.at(biggestFlatSurfaceIndex), min, max);

				if (maxGroundHeight == 0.0f || min[2] < maxGroundHeight)
				//if (false)
				{
					for (unsigned int i = 0; i < clusteredFlatSurfaces.size(); ++i)
					{
						if ((int)i != biggestFlatSurfaceIndex)
						{
							Eigen::Vector4f centroid(0, 0, 0, 1);
							pcl::compute3DCentroid(*cloud, *clusteredFlatSurfaces.at(i), centroid);
							if (maxGroundHeight == 0.0f || centroid[2] <= maxGroundHeight || centroid[2] <= max[2]) // epsilon
							{
								ground = util3d::concatenate(ground, clusteredFlatSurfaces.at(i));
							}
							else if (flatObstacles)
							{
								*flatObstacles = util3d::concatenate(*flatObstacles, clusteredFlatSurfaces.at(i));
							}
						}
					}
				}
				else
				{
					// reject ground!
					ground.reset(new std::vector<int>);
					if (flatObstacles)
					{
						*flatObstacles = flatSurfaces;
					}
				}
			}
		}
		else
		{
			ground = flatSurfaces;
		}

		if (ground->size() != cloud->size())
		{
			// Remove ground
			pcl::IndicesPtr notObstacles = ground;
			if (indices->size())
			{
				notObstacles = util3d::extractIndices(cloud, indices, true);
				notObstacles = util3d::concatenate(notObstacles, ground);
			}
			pcl::IndicesPtr otherStuffIndices = util3d::extractIndices(cloud, notObstacles, true);

			// If ground height is set, remove obstacles under it
			if (maxGroundHeight != 0.0f)
			{
				otherStuffIndices = rtabmap::util3d::passThrough(cloud, otherStuffIndices, "z", maxGroundHeight, std::numeric_limits<float>::max());
			}

			//Cluster remaining stuff (obstacles)
			if (otherStuffIndices->size())
			{
				std::vector<pcl::IndicesPtr> clusteredObstaclesSurfaces = util3d::extractClusters(
					cloud,
					otherStuffIndices,
					clusterRadius,
					minClusterSize);

				// merge indices
				obstacles = util3d::concatenate(clusteredObstaclesSurfaces);
			}
		}
	}
}
