#include "Utils_transform.h"

namespace Utils_transform
{
pcl::PointCloud<pcl::PointXYZ>::Ptr transformPointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const Attitude &transform)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *output, transform.toEigen4f());
    return output;
}

Attitude transformFromTF(const tf::Transform &transform)
{
    Eigen::Affine3d eigenTf;
    tf::transformTFToEigen(transform, eigenTf);
    return Attitude::fromEigen3d(eigenTf);
}
} // namespace Utils_transform