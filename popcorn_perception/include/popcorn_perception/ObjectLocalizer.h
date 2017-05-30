#ifndef __OBJECTLOCALIZER_H__
#define __OBJECTLOCALIZER_H__

#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

#include <popcorn_perception/Kinect.h>
#include <popcorn_perception/Geometry.h>

class ObjectLocalizer
{
  public :

  ObjectLocalizer()
  {
    cloud_pub_initialized = false;
  }


  bool localizePot(tf::Stamped<tf::Pose>& pot_pose,
                   std::string objectFrameIdName);

  bool localizeLid (tf::Stamped<tf::Pose>& lid_pose, 
                    std::string objectFrameIdName);

  bool localizeSmallBowl(tf::Stamped<tf::Pose>& small_bowl_pose,
                         std::string objectFrameIdName);

  bool localizeDeepPlate(tf::Stamped<tf::Pose>& deep_plate_pose,
                         std::string objectFrameIdName);

  bool localizeSaltCellar(tf::Stamped<tf::Pose>& salt_cellar_pose,
                          std::string objectFrameIdName);
 

private:

  void announce_box(const std::string text, 
                    tf::Vector3 min, 
                    tf::Vector3 max, 
                    std::string objectFrameIdName);

  void getPointsInBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                      pcl::PointCloud<pcl::PointXYZRGB>::Ptr inBox,
                      tf::Vector3 min,
                      tf::Vector3 max);

  bool getCirclesFromCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pot_cyl_cloud,
                           double radius_goal,
                           double radius_tolerance,
                           std::vector<tf::Vector3> &center,
                           std::vector<double> &radius,
                           std::vector<int> &numinliers,
                           size_t  iterations,
                           std::string circle_semantics);

  // get average position of points in pointcloud
  bool getCenter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, tf::Vector3 &center);

  bool getHandleRotation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                         tf::Stamped<tf::Pose> &potPose,
                         tf::Vector3 min, tf::Vector3 max);

  // min.z should be the table height, while min->max should span a box where all points necessary for detection lie in
  bool getLidPose(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                  tf::Stamped<tf::Pose> &lid,
                  double radius,
                  tf::Vector3 min,
                  tf::Vector3 max);

  // min.z should be the table height, while min->max should span a box where all points necessary for detection lie in
  bool getPotPoseViaLid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                        tf::Stamped<tf::Pose> &potPose,
                        tf::Vector3 min,
                        tf::Vector3 max);

  // min.z should be the table/drawer height, while min->max should span a box where all points necessary for detection lie in
  bool getCircularObjectPose(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, 
                            tf::Stamped<tf::Pose> &object_pose,
                            double radius_goal, double radius_tolerance,
                            tf::Vector3 min, tf::Vector3 max);

  // publish cloud on debug_cloud topic
  void pubCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

  bool cloud_pub_initialized;
  ros::Publisher cloud_pub;
  std::string frame_id_;


};

#endif

