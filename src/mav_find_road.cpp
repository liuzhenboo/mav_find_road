
#include <PointCloud_Processor.h>

std::string frameId_;
std::string mapFrameId_;
bool waitForTransform_;
tf::TransformListener *tfListener_;
bool mapFrameProjection_;
double pointcloud_x_ = 2.0;
double pointcloud_y_ = 10.0;
double pointcloud_zu_ = 1.0;
double pointcloud_zd_ = 2.0;

std::string configFile_;
ros::Publisher groundPub_;
ros::Publisher obstaclesPub_;
ros::Publisher projObstaclesPub_;
ros::Publisher mapPub_;
ros::Subscriber cloudSub_;
PointCloud_Processor PC_Processor;

rtabmap::Transform transformFromTF(const tf::Transform &transform)
{
	Eigen::Affine3d eigenTf;
	tf::transformTFToEigen(transform, eigenTf);
	return rtabmap::Transform::fromEigen3d(eigenTf);
}

void callback(const sensor_msgs::PointCloud2ConstPtr &cloudMsg)
{

	ros::WallTime time = ros::WallTime::now();

	// ROS_INFO("HELLO");
	if (groundPub_.getNumSubscribers() == 0 && obstaclesPub_.getNumSubscribers() == 0 && projObstaclesPub_.getNumSubscribers() == 0)
	{
		// no one wants the results
		ROS_INFO("no one wants the results!");
		return;
	}

	// localTransform是点云局部坐标系与frameId(base_link)的坐标变换
	rtabmap::Transform localTransform = rtabmap::Transform::getIdentity();
	try
	{
		if (waitForTransform_)
		{
			if (!tfListener_->waitForTransform(frameId_, cloudMsg->header.frame_id, ros::Time(0), ros::Duration(3)))
			{
				ROS_ERROR("Could not get transform from %s to %s after 1 second!", frameId_.c_str(), cloudMsg->header.frame_id.c_str());
				return;
			}
		}
		tf::StampedTransform tmp;
		tfListener_->lookupTransform(frameId_, cloudMsg->header.frame_id, ros::Time(0), tmp);
		localTransform = transformFromTF(tmp);
	}
	catch (tf::TransformException &ex)
	{
		ROS_ERROR("%s", ex.what());
		return;
	}

	// pose指frameId(base_link)在mapFrameId中的姿态
	rtabmap::Transform pose = rtabmap::Transform::getIdentity();
	if (!mapFrameId_.empty())
	{
		try
		{
			if (waitForTransform_)
			{
				if (!tfListener_->waitForTransform(mapFrameId_, frameId_, ros::Time(0), ros::Duration(3)))
				{
					ROS_ERROR("Could not get transform from %s to %s after 1 second!", mapFrameId_.c_str(), frameId_.c_str());
					return;
				}
			}
			tf::StampedTransform tmp;
			tfListener_->lookupTransform(mapFrameId_, frameId_, ros::Time(0), tmp);
			pose = transformFromTF(tmp);
		}
		catch (tf::TransformException &ex)
		{
			ROS_ERROR("%s", ex.what());
			return;
		}
	}

	// 去除无效点云数据
	pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud0(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*cloudMsg, *inputCloud0);
	if (inputCloud0->isOrganized())
	{
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*inputCloud0, *inputCloud0, indices);
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZ>);

	/// TODO: 保留想要范围内的点云
	for (int i = 0; i < inputCloud0->points.size(); i++)
	{
		pcl::PointXYZ pt;
		pt = inputCloud0->points[i];
		if (pt.x < pointcloud_x_ && pt.y < pointcloud_y_ && pt.y > -1.0 * pointcloud_y_ && pt.z > -1.0 * pointcloud_zd_ && pt.z < 1.0 * pointcloud_zu_)
			inputCloud->points.push_back(pt);
	}
	inputCloud->width = inputCloud->points.size();
	inputCloud->height = 1;
	inputCloud->is_dense = true;
	// 主要点云变量
	//Common variables for all strategies
	pcl::IndicesPtr ground, obstacles;
	pcl::PointCloud<pcl::PointXYZ>::Ptr obstaclesCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr groundCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr obstaclesCloudWithoutFlatSurfaces(new pcl::PointCloud<pcl::PointXYZ>);

	// 核心逻辑
	if (inputCloud->size())
	{
		// 转换到base_link坐标系
		inputCloud = rtabmap::util3d::transformPointCloud(inputCloud, localTransform);

		pcl::IndicesPtr flatObstacles(new std::vector<int>);

		// 点云分割
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = PC_Processor.segmentCloud(
			inputCloud,
			pcl::IndicesPtr(new std::vector<int>),
			pose,
			cv::Point3f(localTransform.x(), localTransform.y(), localTransform.z()),
			ground,
			obstacles,
			&flatObstacles);

		// ground与obstacles必须有一个不为空
		if (cloud->size() && ((ground.get() && ground->size()) || (obstacles.get() && obstacles->size())))
		{
			// 得到ground点云
			if (groundPub_.getNumSubscribers() &&
				ground.get() && ground->size())
			{
				pcl::copyPointCloud(*cloud, *ground, *groundCloud);
			}

			if ((obstaclesPub_.getNumSubscribers() || projObstaclesPub_.getNumSubscribers()) && obstacles.get() && obstacles->size())
			{
				// remove flat obstacles from obstacles
				std::set<int> flatObstaclesSet;
				if (projObstaclesPub_.getNumSubscribers())
				{
					flatObstaclesSet.insert(flatObstacles->begin(), flatObstacles->end());
				}

				obstaclesCloud->resize(obstacles->size());
				obstaclesCloudWithoutFlatSurfaces->resize(obstacles->size());

				int oi = 0;
				for (unsigned int i = 0; i < obstacles->size(); ++i)
				{
					obstaclesCloud->points[i] = cloud->at(obstacles->at(i));
					if (flatObstaclesSet.size() == 0 ||
						flatObstaclesSet.find(obstacles->at(i)) == flatObstaclesSet.end())
					{
						obstaclesCloudWithoutFlatSurfaces->points[oi] = obstaclesCloud->points[i];
						obstaclesCloudWithoutFlatSurfaces->points[oi].z = 0;
						++oi;
					}
				}

				obstaclesCloudWithoutFlatSurfaces->resize(oi);
			}

			if (!localTransform.isIdentity() || !pose.isIdentity())
			{
				//transform back in topic frame for 3d clouds and base frame for 2d clouds
				// 在分割的函数中对点云做了一些坐标变换,这里恢复原来的坐标
				float roll, pitch, yaw;
				pose.getEulerAngles(roll, pitch, yaw);
				rtabmap::Transform t = rtabmap::Transform(0, 0, mapFrameProjection_ ? pose.z() : 0, roll, pitch, 0);

				if (obstaclesCloudWithoutFlatSurfaces->size() && !pose.isIdentity())
				{
					obstaclesCloudWithoutFlatSurfaces = rtabmap::util3d::transformPointCloud(obstaclesCloudWithoutFlatSurfaces, t.inverse());

					rtabmap::Transform t1 = rtabmap::Transform(pose.x(), pose.y(), pose.z(), roll, pitch, yaw);
					obstaclesCloudWithoutFlatSurfaces = rtabmap::util3d::transformPointCloud(obstaclesCloudWithoutFlatSurfaces, t1);
				}
				if (groundCloud->size())
				{
					groundCloud = rtabmap::util3d::transformPointCloud(groundCloud, t.inverse());
					rtabmap::Transform t1 = rtabmap::Transform(pose.x(), pose.y(), pose.z(), roll, pitch, yaw);
					groundCloud = rtabmap::util3d::transformPointCloud(groundCloud, t1);
				}
				t = (t * localTransform).inverse();
				// if (groundCloud->size())
				// {
				// 	groundCloud = rtabmap::util3d::transformPointCloud(groundCloud, t);
				// }
				if (obstaclesCloud->size())
				{
					obstaclesCloud = rtabmap::util3d::transformPointCloud(obstaclesCloud, t);
				}
			}
		}
		else
		{
			ROS_INFO("no_pointcloud");
		}
	}
	else
	{
		ROS_WARN("mav_find_road: Input cloud is empty! (%d x %d, is_dense=%d)", cloudMsg->width, cloudMsg->height, cloudMsg->is_dense ? 1 : 0);
	}

	if (groundPub_.getNumSubscribers())
	{
		sensor_msgs::PointCloud2 rosCloud;
		pcl::toROSMsg(*groundCloud, rosCloud);
		//rosCloud.header = cloudMsg->header;
		rosCloud.header.stamp = cloudMsg->header.stamp;
		rosCloud.header.frame_id = mapFrameId_;
		//publish the message
		groundPub_.publish(rosCloud);
	}

	if (obstaclesPub_.getNumSubscribers())
	{
		sensor_msgs::PointCloud2 rosCloud;
		pcl::toROSMsg(*obstaclesCloud, rosCloud);
		rosCloud.header = cloudMsg->header;

		//publish the message
		obstaclesPub_.publish(rosCloud);
	}

	if (projObstaclesPub_.getNumSubscribers())
	{
		sensor_msgs::PointCloud2 rosCloud;
		pcl::toROSMsg(*obstaclesCloudWithoutFlatSurfaces, rosCloud);
		rosCloud.header.stamp = cloudMsg->header.stamp;
		rosCloud.header.frame_id = mapFrameId_;
		// ROS_INFO("LIUZHENBO");
		//publish the message
		projObstaclesPub_.publish(rosCloud);
	}
	// if (mapPub_.getNumSubscribers())
	if (false)
	{
		sensor_msgs::PointCloud2 rosCloud;
		pcl::toROSMsg(*obstaclesCloudWithoutFlatSurfaces, rosCloud);
		rosCloud.header.stamp = cloudMsg->header.stamp;
		rosCloud.header.frame_id = frameId_;

		//publish the message
		projObstaclesPub_.publish(rosCloud);
	}

	ROS_DEBUG("Obstacles segmentation time = %f s", (ros::WallTime::now() - time).toSec());
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mav_find_road");
	ros::NodeHandle nh("~");
	tfListener_ = new tf::TransformListener();
	nh.param("frame_id", frameId_, frameId_);
	nh.param("map_frame_id", mapFrameId_, mapFrameId_);
	nh.param("wait_for_transform", waitForTransform_, waitForTransform_);
	nh.param("mapFrameProjection", mapFrameProjection_, mapFrameProjection_);
	nh.param("pointcloud_x", pointcloud_x_, pointcloud_x_);
	nh.param("pointcloud_y", pointcloud_y_, pointcloud_y_);
	nh.param("pointcloud_zu", pointcloud_zu_, pointcloud_zu_);
	nh.param("pointcloud_zd", pointcloud_zd_, pointcloud_zd_);

	PC_Processor.pc_init();

	nh.param("preVoxelFiltering", PC_Processor.preVoxelFiltering_, PC_Processor.preVoxelFiltering_);
	// nh.param("projMapFrame", PC_Processor.projMapFrame_, PC_Processor.projMapFrame_);
	PC_Processor.projMapFrame_ = mapFrameProjection_;
	nh.param("normalsSegmentation", PC_Processor.normalsSegmentation_, PC_Processor.normalsSegmentation_);
	nh.param("groundIsObstacle", PC_Processor.groundIsObstacle_, PC_Processor.groundIsObstacle_);
	nh.param("flatObstaclesDetected", PC_Processor.flatObstaclesDetected_,
			 PC_Processor.flatObstaclesDetected_);
	nh.param("minClusterSize", PC_Processor.minClusterSize_, PC_Processor.minClusterSize_);
	nh.param("noiseFilteringMinNeighbors", PC_Processor.cellSize_, PC_Processor.cellSize_);
	nh.param("normalKSearch", PC_Processor.normalKSearch_, PC_Processor.normalKSearch_);

	// double to float
	nh.param("cellSize", PC_Processor.cellSize, PC_Processor.cellSize);
	nh.param("footprintLength", PC_Processor.footprintLength, PC_Processor.footprintLength);
	nh.param("footprintWidth", PC_Processor.footprintWidth, PC_Processor.footprintWidth);
	nh.param("footprintHeight", PC_Processor.footprintHeight, PC_Processor.footprintHeight);
	nh.param("maxGroundHeight", PC_Processor.maxGroundHeight, PC_Processor.maxGroundHeight);
	nh.param("maxGroundAngle", PC_Processor.maxGroundAngle, PC_Processor.maxGroundAngle);
	nh.param("minGroundHeight", PC_Processor.minGroundHeight, PC_Processor.minGroundHeight);
	nh.param("clusterRadius", PC_Processor.clusterRadius, PC_Processor.clusterRadius);
	nh.param("noiseFilteringRadius", PC_Processor.noiseFilteringRadius, PC_Processor.noiseFilteringRadius);
	nh.param("maxObstacleHeight", PC_Processor.maxObstacleHeight, PC_Processor.maxObstacleHeight);

	PC_Processor.double2float();
	if (mapFrameProjection_ && mapFrameId_.empty())
	{
		ROS_ERROR("mav_find_road: Parameter mapFrameProjection is true but map_frame_id is not set!");
	}
	double a = 0.1;
	float b = 0.2;
	b = a;
	std::cout << b << std::endl;
	ROS_INFO("HELLO11");
	cloudSub_ = nh.subscribe("/camera/depth/color/points", 1, callback);

	groundPub_ = nh.advertise<sensor_msgs::PointCloud2>("ground", 1);
	obstaclesPub_ = nh.advertise<sensor_msgs::PointCloud2>("obstacles", 1);
	projObstaclesPub_ = nh.advertise<sensor_msgs::PointCloud2>("proj_obstacles", 1);

	ros::Rate rate(10);
	bool status = ros::ok();
	while (status)
	{
		ros::spinOnce();
		status = ros::ok();
		rate.sleep();
	}
	return 0;
}