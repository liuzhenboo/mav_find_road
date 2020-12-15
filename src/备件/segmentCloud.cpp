
#include <rtabmap/core/util3d_mapping.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/utilite/ULogger.h>

PointCloud_Processor::PointCloud_Processor(/* args */)
{
}

PointCloud_Processor::~PointCloud_Processor()
{
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud_Processor::segmentCloud(
	const pcl::PointCloud<Pcl::PointXYZ>::Ptr &cloudIn,
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

 	pcl::PointCloud<Pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<Pcl::PointXYZ>);
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
			util3d::segmentObstaclesFromGround<Pcl::PointXYZ>(
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
