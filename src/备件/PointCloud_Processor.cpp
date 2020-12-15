
#include <rtabmap/core/util3d_mapping.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/utilite/ULogger.h>

PointCloud_Processor::segmentCloud(/* args */)
{
}

PointCloud_Processor::~segmentCloud()
{
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud_Processor::TransformPointCloud(
	const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
	const Transform &transform)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*cloud, *output, transform.toEigen4f());
	return output;
}

pcl::IndicesPtr radiusFilteringImpl(
	const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
	const pcl::IndicesPtr &indices,
	float radiusSearch,
	int minNeighborsInRadius)
{
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>(false));

	if (indices->size())
	{
		pcl::IndicesPtr output(new std::vector<int>(indices->size()));
		int oi = 0; // output iterator
		tree->setInputCloud(cloud, indices);
		for (unsigned int i = 0; i < indices->size(); ++i)
		{
			std::vector<int> kIndices;
			std::vector<float> kDistances;
			int k = tree->radiusSearch(cloud->at(indices->at(i)), radiusSearch, kIndices, kDistances);
			if (k > minNeighborsInRadius)
			{
				output->at(oi++) = indices->at(i);
			}
		}
		output->resize(oi);
		return output;
	}
	else
	{
		pcl::IndicesPtr output(new std::vector<int>(cloud->size()));
		int oi = 0; // output iterator
		tree->setInputCloud(cloud);
		for (unsigned int i = 0; i < cloud->size(); ++i)
		{
			std::vector<int> kIndices;
			std::vector<float> kDistances;
			int k = tree->radiusSearch(cloud->at(i), radiusSearch, kIndices, kDistances);
			if (k > minNeighborsInRadius)
			{
				output->at(oi++) = i;
			}
		}
		output->resize(oi);
		return output;
	}
}

pcl::PointCloud<pcl::PointXYZ>::Ptr voxelizeImpl(
	const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
	const pcl::IndicesPtr &indices,
	float voxelSize)
{
	UASSERT(voxelSize > 0.0f);
	pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
	if ((cloud->is_dense && cloud->size()) || (!cloud->is_dense && indices->size()))
	{
		pcl::VoxelGrid<pcl::PointXYZ> filter;
		filter.setLeafSize(voxelSize, voxelSize, voxelSize);
		filter.setInputCloud(cloud);
		if (indices->size())
		{
			filter.setIndices(indices);
		}
		filter.filter(*output);
	}
	else if (cloud->size() && !cloud->is_dense && indices->size() == 0)
	{
		UWARN("Cannot voxelize a not dense (organized) cloud with empty indices! (input=%d pts). Returning empty cloud!", (int)cloud->size());
	}
	return output;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud_Processor::segmentCloud(
	const pcl::PointCloud<PointXYZ>::Ptr &cloudIn,
	const pcl::IndicesPtr &indicesIn,
	const Transform &pose,
	const cv::Point3f &viewPoint,
	pcl::IndicesPtr &groundIndices,
	pcl::IndicesPtr &obstaclesIndices,
	pcl::IndicesPtr *flatObstacles) const
{
	groundIndices.reset(new std::vector<int>);
	obstaclesIndices.reset(new std::vector<int>);
	if (flatObstacles)
	{
		flatObstacles->reset(new std::vector<int>);
	}

	pcl::PointCloud<PointXYZ>::Ptr cloud(new pcl::PointCloud<PointXYZ>);
	pcl::IndicesPtr indices(new std::vector<int>);

	if (preVoxelFiltering_)
	{
		// voxelize to grid cell size
		cloud = voxelizeImpl(cloudIn, indicesIn, cellSize_);

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
	cloud = TransformPointCloud(cloud, Transform(0, 0, projMapFrame_ ? pose.z() : 0, roll, pitch, 0));

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
			util3d::segmentObstaclesFromGround<pcl::PointXYZ>(
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
		// else
		// {
		// 	UDEBUG("");
		// 	// passthrough filter
		// 	groundIndices = rtabmap::util3d::passThrough(cloud, indices, "z",
		// 												 minGroundHeight_ != 0.0f ? minGroundHeight_ : std::numeric_limits<int>::min(),
		// 												 maxGroundHeight_ != 0.0f ? maxGroundHeight_ : std::numeric_limits<int>::max());

		// 	pcl::IndicesPtr notObstacles = groundIndices;
		// 	if (indices->size())
		// 	{
		// 		notObstacles = util3d::extractIndices(cloud, indices, true);
		// 		notObstacles = util3d::concatenate(notObstacles, groundIndices);
		// 	}
		// 	obstaclesIndices = rtabmap::util3d::extractIndices(cloud, notObstacles, true);
		// }

		UDEBUG("groundIndices=%d obstaclesIndices=%d", (int)groundIndices->size(), (int)obstaclesIndices->size());

		// Do radius filtering after voxel filtering ( a lot faster)
		if (noiseFilteringRadius_ > 0.0 && noiseFilteringMinNeighbors_ > 0)
		{
			UDEBUG("");
			if (groundIndices->size())
			{
				groundIndices = radiusFilteringImpl(cloud, groundIndices, noiseFilteringRadius_, noiseFilteringMinNeighbors_);
			}
			if (obstaclesIndices->size())
			{
				obstaclesIndices = radiusFilteringImpl(cloud, obstaclesIndices, noiseFilteringRadius_, noiseFilteringMinNeighbors_);
			}
			if (flatObstacles && (*flatObstacles)->size())
			{
				*flatObstacles = radiusFilteringImpl(cloud, *flatObstacles, noiseFilteringRadius_, noiseFilteringMinNeighbors_);
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

void segmentObstaclesFromGround(
	const  pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
	const  pcl::IndicesPtr &indices,
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
			std::vector<pcl::IndicesPtr> clusteredFlatSurfaces = extractClusters(
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
