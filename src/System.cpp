
#include <System.h>

System::System()
{
	std::string frameId_;
	std::string mapFrameId_;
	bool waitForTransform_;
	tf::TransformListener *tfListener_;
	bool mapFrameProjection_;
	double pointcloud_xu_ = 5.0;
	double pointcloud_xd_ = -5.0;

	double pointcloud_yu_ = 5.0;
	double pointcloud_yd_ = -5.0;

	double pointcloud_zu_ = 5.0;
	double pointcloud_zd_ = -5.0;
	system_status_ = 0;
}
System::~System()
{
}
void System::Init_parameter(ros::NodeHandle &nh)
{

	PC_Processor_.pc_init(nh);
	map_.init(nh);
	tfListener_ = new tf::TransformListener();
	mapFrameProjection_ = false;
	nh.param("frame_id", frameId_, frameId_);
	nh.param("map_frame_id", mapFrameId_, mapFrameId_);
	nh.param("wait_for_transform", waitForTransform_, waitForTransform_);
	nh.param("projMapFrame", mapFrameProjection_, mapFrameProjection_);
	nh.param("pointcloud_xu", pointcloud_xu_, pointcloud_xu_);
	nh.param("pointcloud_xd", pointcloud_xd_, pointcloud_xd_);
	nh.param("pointcloud_yu", pointcloud_yu_, pointcloud_yu_);
	nh.param("pointcloud_yd", pointcloud_yd_, pointcloud_yd_);
	nh.param("pointcloud_zu", pointcloud_zu_, pointcloud_zu_);
	nh.param("pointcloud_zd", pointcloud_zd_, pointcloud_zd_);
	if (mapFrameProjection_ && mapFrameId_.empty())
	{
		ROS_ERROR("mav_find_road: Parameter mapFrameProjection is true but map_frame_id is not set!");
	}
	double a = 0.1;
	float b = 0.2;
	b = a;
	std::cout << b << std::endl;
	ROS_INFO("Starting...");

	// 需要订阅点云数据
	cloudSub_ = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/color/points", 1, boost::bind(&System::callback, this, _1));

	// 发布道路点云数据
	groundPub_ = nh.advertise<sensor_msgs::PointCloud2>("ground", 1);
	localgroundPub_ = nh.advertise<sensor_msgs::PointCloud2>("localground", 1);

	obstaclesPub_ = nh.advertise<sensor_msgs::PointCloud2>("obstacles", 1);
	projObstaclesPub_ = nh.advertise<sensor_msgs::PointCloud2>("proj_obstacles", 1);
	unsurePub_ = nh.advertise<sensor_msgs::PointCloud2>("unsure", 1);

	// 发布道路信息
	//.......
}

void System::run()
{
	ros::Rate rate(10);
	bool status = ros::ok();
	while (status)
	{
		ros::spinOnce();
		status = ros::ok();
		rate.sleep();
	}
}
void System::callback(const sensor_msgs::PointCloud2ConstPtr &cloudMsg)
{

	ros::WallTime time = ros::WallTime::now();
	if (groundPub_.getNumSubscribers() == 0 && obstaclesPub_.getNumSubscribers() == 0)
	{
		// 无人订阅,则不进行处理.
		ROS_INFO("no one wants the results!");
		return;
	}

	// tf树获得localTransform: 点云坐标系与机体系frameId(base_link)的坐标变换
	Attitude localTransform = Attitude::getIdentity();
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
		localTransform = Utils_transform::transformFromTF(tmp);
	}
	catch (tf::TransformException &ex)
	{
		ROS_ERROR("%s", ex.what());
		return;
	}

	// // tf树获得pose: frameId(base_link)在mapFrameId中的姿态
	Attitude pose = Attitude::getIdentity();
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
			pose = Utils_transform::transformFromTF(tmp);
			pose_ = pose;
			map_.SetPose(pose);
		}
		catch (tf::TransformException &ex)
		{
			ROS_ERROR("%s", ex.what());
			return;
		}
	}

	// 去除无效点云数据
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud0(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::fromROSMsg(*cloudMsg, *inputCloud0);
	if (inputCloud0->isOrganized())
	{
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*inputCloud0, *inputCloud0, indices);
	}

	// 主要的点云变量
	pcl::IndicesPtr ground, obstacles;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr obstaclesCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr groundCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr obstaclesCloudWithoutFlatSurfaces(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr debug_pt(new pcl::PointCloud<pcl::PointXYZRGB>);

	// 核心逻辑,对inputCloud进行处理
	if (inputCloud0->size())
	{
		// 转换到base_link坐标系,
		inputCloud0 = Utils_transform::transformPointCloud(inputCloud0, localTransform);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		debug_pt = inputCloud;
		///  保留想要范围内的点云
		for (int i = 0; i < inputCloud0->points.size(); i++)
		{
			pcl::PointXYZRGB pt;
			pt = inputCloud0->points[i];
			if (pt.x < pointcloud_xu_ && pt.x > pointcloud_xd_ && pt.y < pointcloud_yu_ && pt.y > pointcloud_yd_ && pt.z > pointcloud_zd_ && pt.z < pointcloud_zu_)
				inputCloud->points.push_back(pt);
		}
		inputCloud->width = inputCloud->points.size();
		inputCloud->height = 1;
		inputCloud->is_dense = true;

		if (!inputCloud->size())
			std::cout << "点云切块后为空！" << std::endl;

		pcl::IndicesPtr flatObstacles(new std::vector<int>);

		// 点云分割
		// ground为道路点云
		// obstacles为影响行军的障碍物点云
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = PC_Processor_.segmentCloud(
			inputCloud,
			pcl::IndicesPtr(new std::vector<int>),
			pose,
			cv::Point3f(localTransform.x(), localTransform.y(), localTransform.z()),
			ground,
			obstacles,
			&flatObstacles);

		//debug
		// if (cloud->size())
		// {
		// 	std::cout << "非空" << std::endl;
		// }

		// ground与obstacles必须有一个不为空
		if (cloud->size() && ((ground.get() && ground->size()) || (obstacles.get() && obstacles->size())))
		{
			// 对道路点云索引处理
			// 从索引ground得到道路真实数据groundCloud
			if (groundPub_.getNumSubscribers() &&
				ground.get() && ground->size())
			{
				pcl::copyPointCloud(*cloud, *ground, *groundCloud);
			}

			// 对障碍物点云索引处理
			if (obstaclesPub_.getNumSubscribers() || obstacles.get() && obstacles->size())
			{
				obstaclesCloud->resize(obstacles->size());
				for (unsigned int i = 0; i < obstacles->size(); ++i)
				{
					obstaclesCloud->points[i] = cloud->at(obstacles->at(i));
				}
			}

			if (!localTransform.isIdentity() || !pose.isIdentity())
			{
				//transform back in topic frame for 3d clouds and base frame for 2d clouds
				// 在分割的函数中对点云做了一些坐标变换,这里恢复原来的坐标
				float roll, pitch, yaw;
				pose.getEulerAngles(roll, pitch, yaw);
				Attitude t = Attitude(0, 0, mapFrameProjection_ ? pose.z() : 0, roll, pitch, 0);

				if (groundCloud->size())
				{
					groundCloud = Utils_transform::transformPointCloud(groundCloud, t.inverse());
					Attitude t1 = Attitude(pose.x(), pose.y(), pose.z(), roll, pitch, yaw);
					groundCloud = Utils_transform::transformPointCloud(groundCloud, t1);
					//debug_pt = Utils_transform::transformPointCloud(debug_pt, t1);
				}
				//t = (t * localTransform).inverse();

				if (obstaclesCloud->size())
				{
					obstaclesCloud = Utils_transform::transformPointCloud(obstaclesCloud, t.inverse());
					Attitude t2 = Attitude(pose.x(), pose.y(), pose.z(), roll, pitch, yaw);
					obstaclesCloud = Utils_transform::transformPointCloud(obstaclesCloud, t2);
				}
				else
				{
					std::cout << "没有检测到障碍物！" << std::endl;
				}
			}
		}
		else
		{
			ROS_INFO("no ground pointcloud and obstacles pointcloud!");
		}
	}
	else
	{
		ROS_WARN("mav_find_road: Input cloud is empty! (%d x %d, is_dense=%d)", cloudMsg->width, cloudMsg->height, cloudMsg->is_dense ? 1 : 0);
	}

	// if (groundPub_.getNumSubscribers())
	// {
	// 	sensor_msgs::PointCloud2 rosCloud;
	// 	pcl::toROSMsg(*groundCloud, rosCloud);
	// 	//rosCloud.header = cloudMsg->header;
	// 	rosCloud.header.stamp = cloudMsg->header.stamp;
	// 	rosCloud.header.frame_id = mapFrameId_;
	// 	//publish the message
	// 	groundPub_.publish(rosCloud);
	// }

	// if (obstaclesPub_.getNumSubscribers())
	// {
	// 	sensor_msgs::PointCloud2 rosCloud;
	// 	pcl::toROSMsg(*obstaclesCloud, rosCloud);
	// 	rosCloud.header.stamp = cloudMsg->header.stamp;
	// 	rosCloud.header.frame_id = mapFrameId_;
	// 	//publish the message
	// 	obstaclesPub_.publish(rosCloud);
	// }
	ROS_DEBUG("road detect cost = %f s", (ros::WallTime::now() - time).toSec());
	Sent2MapHandle(groundCloud, obstaclesCloud);
}

void System::Sent2MapHandle(pcl::PointCloud<pcl::PointXYZRGB>::Ptr groundCloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr obstaclesCloud)
{
	ros::WallTime time1 = ros::WallTime::now();
	// 融合当前道路点云
	//map_.fusion(groundCloud);
	// 初始化，丢失状态
	if (system_status_ == 0)
	{
		system_status_ = map_.Initialization_Newground(groundCloud, obstaclesCloud);
		if (system_status_ == 1)
		{
			std::cout << "初始化成功！" << std::endl;
		}
		else
		{
			//std::cout << "正在初始化．．．"　<< std::endl;
		}
	}
	// 初始化成功
	else
	{
		// 跟踪成功
		if (map_.Track(groundCloud))
		{
			map_.Fusion2Localmap(groundCloud, obstaclesCloud);
			map_.Add2Globalmap();
			std::cout << "fusion_cell" << std::endl;
			vis_map();
		}
		else
		{
			//std::cout << "跟踪失败！重新初始化！！"　<< std::endl;
			system_status_ = 0;
			map_.SetInitFlag(1);
		}
	}
	ROS_DEBUG("map fusion cost = %f s", (ros::WallTime::now() - time1).toSec());
	obstaclesCloud_ = groundCloud;
}

void System::vis_map()
{
	if (groundPub_.getNumSubscribers())
	{
		sensor_msgs::PointCloud2 rosCloud;
		sensor_msgs::PointCloud2 roslocalCloud;
		sensor_msgs::PointCloud2 rosobsCloud;
		sensor_msgs::PointCloud2 rosunsureCloud;

		pcl::PointCloud<pcl::PointXYZ>::Ptr all_groundCloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr local_groundCloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr obsCloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr unsureCloud(new pcl::PointCloud<pcl::PointXYZ>);

		//pcl::PointCloud<pcl::PointXYZRGB>::Ptr all_groundCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

		//all_groundCloud = map_.get_allmap();
		all_groundCloud = map_.get_cellMap();
		local_groundCloud = map_.get_localMap();
		obsCloud = map_.get_obsMap();
		unsureCloud = map_.get_unsureMap();

		pcl::toROSMsg(*all_groundCloud, rosCloud);
		pcl::toROSMsg(*local_groundCloud, roslocalCloud);
		pcl::toROSMsg(*obsCloud, rosobsCloud);
		pcl::toROSMsg(*obstaclesCloud_, rosunsureCloud);

		//rosCloud.header = cloudMsg->header;
		//rosCloud.header.stamp = cloudMsg_->header.stamp;
		rosCloud.header.frame_id = mapFrameId_;
		roslocalCloud.header.frame_id = mapFrameId_;
		rosobsCloud.header.frame_id = mapFrameId_;
		rosunsureCloud.header.frame_id = mapFrameId_;

		//publish the message
		groundPub_.publish(rosCloud);
		localgroundPub_.publish(roslocalCloud);
		obstaclesPub_.publish(rosobsCloud);
		unsurePub_.publish(rosunsureCloud);
	}
	// map_.fusion_cell(obstaclesCloud, 2);

	// if (obstaclesPub_.getNumSubscribers())
	// {
	// 	sensor_msgs::PointCloud2 rosCloud;
	// 	pcl::PointCloud<pcl::PointXYZ>::Ptr all_obsCloud(new pcl::PointCloud<pcl::PointXYZ>);
	// 	all_obsCloud = map_.get_ObscellMap();
	// 	pcl::toROSMsg(*all_obsCloud, rosCloud);
	// 	rosCloud.header.stamp = cloudMsg_->header.stamp;
	// 	rosCloud.header.frame_id = mapFrameId_;
	// 	//publish the message
	// 	obstaclesPub_.publish(rosCloud);
	// }
}