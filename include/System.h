
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/Transform.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <boost/mpl/bind.hpp>
#include <PC_Segment.h>

class System
{
public:
	System();
	~System();
	void callback(const sensor_msgs::PointCloud2ConstPtr &cloudMsg);
	void Init_parameter(ros::NodeHandle &);
	void run();

private:
	std::string frameId_;
	std::string mapFrameId_;
	bool waitForTransform_;
	tf::TransformListener *tfListener_;
	bool mapFrameProjection_;
	double pointcloud_x_ = 2.0;
	double pointcloud_y_ = 10.0;
	double pointcloud_zu_ = 1.0;
	double pointcloud_zd_ = 2.0;

	ros::Publisher groundPub_;
	ros::Publisher obstaclesPub_;
	ros::Publisher projObstaclesPub_;

	ros::Subscriber cloudSub_;

	// 点云切割器
	PC_Segment PC_Processor_;
};
