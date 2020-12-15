
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

class ObstaclesDetection
{
public:
	ObstaclesDetection() : frameId_("base_link"),
						   waitForTransform_(false),
						   mapFrameProjection_(true)
	{
	}

	~ObstaclesDetection()
	{
	}

	rtabmap::Transform transformFromTF(const tf::Transform &transform)
	{
		Eigen::Affine3d eigenTf;
		tf::transformTFToEigen(transform, eigenTf);
		return rtabmap::Transform::fromEigen3d(eigenTf);
	}

	void Init(int argc, char **argv)
	{
		ros::NodeHandle nh("~");

		nh.param("frame_id", frameId_, frameId_);
		nh.param("map_frame_id", mapFrameId_, mapFrameId_);
		nh.param("wait_for_transform", waitForTransform_, waitForTransform_);
		nh.param("mapFrameProjection", mapFrameProjection_, mapFrameProjection_);

		if (mapFrameProjection_ && mapFrameId_.empty())
		{
			ROS_ERROR("obstacles_detection: Parameter mapFrameProjection is true but map_frame_id is not set!");
		}

		//grid_.parseParameters(parameters);
		ROS_INFO("HELLO11");

		cloudSub_ = nh.subscribe("/camera/depth/color/points", 1, &ObstaclesDetection::callback, this);

		groundPub_ = nh.advertise<sensor_msgs::PointCloud2>("ground", 1);
		obstaclesPub_ = nh.advertise<sensor_msgs::PointCloud2>("obstacles", 1);
		projObstaclesPub_ = nh.advertise<sensor_msgs::PointCloud2>("proj_obstacles", 1);
	}

	void callback(const sensor_msgs::PointCloud2ConstPtr &cloudMsg)
	{
		ros::WallTime time = ros::WallTime::now();
		ROS_INFO("HELLO");
		if (groundPub_.getNumSubscribers() == 0 && obstaclesPub_.getNumSubscribers() == 0 && projObstaclesPub_.getNumSubscribers() == 0)
		{
			// no one wants the results
			return;
		}

		// localTransform是点云局部坐标系与frameId(base_link)的坐标变换
		rtabmap::Transform localTransform = rtabmap::Transform::getIdentity();
		try
		{
			if (waitForTransform_)
			{
				if (!tfListener_.waitForTransform(frameId_, cloudMsg->header.frame_id, cloudMsg->header.stamp, ros::Duration(1)))
				{
					ROS_ERROR("Could not get transform from %s to %s after 1 second!", frameId_.c_str(), cloudMsg->header.frame_id.c_str());
					return;
				}
			}
			tf::StampedTransform tmp;
			tfListener_.lookupTransform(frameId_, cloudMsg->header.frame_id, cloudMsg->header.stamp, tmp);
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
					if (!tfListener_.waitForTransform(mapFrameId_, frameId_, cloudMsg->header.stamp, ros::Duration(1)))
					{
						ROS_ERROR("Could not get transform from %s to %s after 1 second!", mapFrameId_.c_str(), frameId_.c_str());
						return;
					}
				}
				tf::StampedTransform tmp;
				tfListener_.lookupTransform(mapFrameId_, frameId_, cloudMsg->header.stamp, tmp);
				pose = transformFromTF(tmp);
			}
			catch (tf::TransformException &ex)
			{
				ROS_ERROR("%s", ex.what());
				return;
			}
		}

		// 去除无效点云数据
		pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg(*cloudMsg, *inputCloud);
		if (inputCloud->isOrganized())
		{
			std::vector<int> indices;
			pcl::removeNaNFromPointCloud(*inputCloud, *inputCloud, indices);
		}

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
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = grid_.segmentCloud<pcl::PointXYZ>(
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

				if ((obstaclesPub_.getNumSubscribers() || projObstaclesPub_.getNumSubscribers()) &&
					obstacles.get() && obstacles->size())
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
					}

					t = (t * localTransform).inverse();
					if (groundCloud->size())
					{
						groundCloud = rtabmap::util3d::transformPointCloud(groundCloud, t);
					}
					if (obstaclesCloud->size())
					{
						obstaclesCloud = rtabmap::util3d::transformPointCloud(obstaclesCloud, t);
					}
				}
			}
		}
		else
		{
			ROS_WARN("obstacles_detection: Input cloud is empty! (%d x %d, is_dense=%d)", cloudMsg->width, cloudMsg->height, cloudMsg->is_dense ? 1 : 0);
		}

		if (groundPub_.getNumSubscribers())
		{
			sensor_msgs::PointCloud2 rosCloud;
			pcl::toROSMsg(*groundCloud, rosCloud);
			rosCloud.header = cloudMsg->header;

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
			rosCloud.header.frame_id = frameId_;

			//publish the message
			projObstaclesPub_.publish(rosCloud);
		}

		ROS_DEBUG("Obstacles segmentation time = %f s", (ros::WallTime::now() - time).toSec());
	}

private:
	std::string frameId_;
	std::string mapFrameId_;
	bool waitForTransform_;

	rtabmap::OccupancyGrid grid_;
	bool mapFrameProjection_;

	tf::TransformListener tfListener_;

	ros::Publisher groundPub_;
	ros::Publisher obstaclesPub_;
	ros::Publisher projObstaclesPub_;

	ros::Subscriber cloudSub_;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "obstacles_detection");

	ObstaclesDetection Sys_;
	Sys_.Init(argc, argv);
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