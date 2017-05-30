#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>

#include <pcl/ros/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <popcorn_perception/Kinect.h>
#include <popcorn_perception/Geometry.h>

Kinect *Kinect::instance_ = 0;

void Kinect::getCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string frame_id)
{
  sensor_msgs::PointCloud2 pc;

  pc  = *(ros::topic::waitForMessage<sensor_msgs::PointCloud2>(rgb_topic_));

  // TODO(lisca): Find why sometimes we don't get the oldest point cloud!
  pc  = *(ros::topic::waitForMessage<sensor_msgs::PointCloud2>(rgb_topic_));


  tf::Stamped<tf::Pose> net_stamped = Geometry::getPose(frame_id.c_str(), rgb_optical_frame_.c_str());

  tf::Transform net_transform;

  net_transform.setOrigin(net_stamped.getOrigin());
  net_transform.setRotation(net_stamped.getRotation());

  sensor_msgs::PointCloud2 pct;

  pcl_ros::transformPointCloud(frame_id.c_str(), net_transform, pc, pct);

  pct.header.frame_id = frame_id.c_str();

  pcl::fromROSMsg(pct, *cloud);
}

/*
void Kinect::getCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string frame_id, ros::Time after, ros::Time *tm)
{
  sensor_msgs::PointCloud2 pc;
  bool found = false;
  while (!found)
  {
    pc  = *(ros::topic::waitForMessage<sensor_msgs::PointCloud2>(rgb_topic_));

    if ((after == ros::Time(0,0)) || (pc.header.stamp > after))
    {
      found = true;
    }
    else
    {
      ROS_ERROR("getKinectCloudXYZ cloud too old : stamp %f , target time %f",pc.header.stamp.toSec(), after.toSec());
    }
  }

  if (tm)
  {
    *tm = pc.header.stamp;
  }
 
  tf::Stamped<tf::Pose> net_stamped = Geometry::getPose(frame_id.c_str(), rgb_optical_frame_.c_str());
  tf::Transform net_transform;

  net_transform.setOrigin(net_stamped.getOrigin());
  net_transform.setRotation(net_stamped.getRotation());

  sensor_msgs::PointCloud2 pct;

  pcl_ros::transformPointCloud(frame_id.c_str(), net_transform, pc, pct);
  pct.header.frame_id = frame_id.c_str();

  pcl::fromROSMsg(pct, *cloud);
}
*/
