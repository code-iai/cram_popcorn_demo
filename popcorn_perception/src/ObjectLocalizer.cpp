#include <popcorn_perception/ObjectLocalizer.h>


bool ObjectLocalizer::localizePot (tf::Stamped<tf::Pose>& pot_pose, std::string objectFrameIdName)
{
  //get the pot box size
  std::vector<double> voiMin, voiMax;
  ros::NodeHandle node_handle;
  node_handle.getParam("pot_voi_min", voiMin);
  node_handle.getParam("pot_voi_max", voiMax);
  ROS_INFO("POT: Min and max points: %f %f %f, %f %f %f", voiMin[0], voiMin[1], voiMin[2], voiMax[0], voiMax[1], voiMax[2]);

  tf::Vector3 minP, maxP;
  minP.setValue(voiMin[0], voiMin[1], voiMin[2]);
  maxP.setValue(voiMax[0], voiMax[1], voiMax[2]);

  //announce the point cloud box to be visualized in rviz
  announce_box("Pot", minP, maxP, objectFrameIdName);
  
  //get the point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  Kinect::getInstance()->getCloud(cloud, objectFrameIdName); 
 
 //detect the pose, using the lid detection
  bool found = getPotPoseViaLid(cloud, pot_pose, minP, maxP);
  if (!found)
    return false;

  return true;
}

bool ObjectLocalizer::localizeLid (tf::Stamped<tf::Pose>& lid_pose, std::string objectFrameIdName)
{
  //get the pot box size
  std::vector<double> voiMin, voiMax;
  double lid_radius_expected;
  ros::NodeHandle node_handle;
  node_handle.getParam("lid_voi_min", voiMin);
  node_handle.getParam("lid_voi_max", voiMax);
  node_handle.getParam("lid_radius_expected", lid_radius_expected);
  ROS_INFO("LID: Min and max points: %f %f %f, %f %f %f", voiMin[0], voiMin[1], voiMin[2], voiMax[0], voiMax[1], voiMax[2]);

  tf::Vector3 minP, maxP;
  minP.setValue(voiMin[0], voiMin[1], voiMin[2]);
  maxP.setValue(voiMax[0], voiMax[1], voiMax[2]);

  //announce the point cloud box to be visualized in rviz
  announce_box("Lid", minP, maxP, objectFrameIdName);
  
  //get the point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  Kinect::getInstance()->getCloud(cloud, objectFrameIdName); 
 
 //detect the pose, using the lid detection
  bool found = getLidPose(cloud, lid_pose, lid_radius_expected, minP, maxP);
  if (!found)
    return false;

  return true;
}

bool ObjectLocalizer::localizeSmallBowl(tf::Stamped<tf::Pose>& small_bowl_pose, std::string objectFrameIdName)
{
  //get the small bowl dimensions
  double small_bowl_radius, small_bowl_tolerance;  
  std::vector<double> voiMin, voiMax;
  ros::NodeHandle node_handle;

  node_handle.getParam("small_bowl_radius", small_bowl_radius);
  node_handle.getParam("small_bowl_tolerance", small_bowl_tolerance);
  node_handle.getParam("small_bowl_voi_min", voiMin);
  node_handle.getParam("small_bowl_voi_max", voiMax);
  ROS_INFO("Small Bowl: Min and max points: %f %f %f, %f %f %f", voiMin[0], voiMin[1], voiMin[2], voiMax[0], voiMax[1], voiMax[2]);

  tf::Vector3 minP, maxP;
  minP.setValue(voiMin[0], voiMin[1], voiMin[2]);
  maxP.setValue(voiMax[0], voiMax[1], voiMax[2]);

  //announce the point cloud box to be visualized in rviz
  announce_box("Small Bowl", minP, maxP, objectFrameIdName);

  //get the point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  Kinect::getInstance()->getCloud(cloud, objectFrameIdName);

  //get the small bowl pose
  bool found = getCircularObjectPose(cloud, small_bowl_pose, small_bowl_radius, small_bowl_tolerance, minP, maxP);
  if (!found)
    return false;

  return true;
}

bool ObjectLocalizer::localizeDeepPlate(tf::Stamped<tf::Pose>& deep_plate_pose, std::string objectFrameIdName)
{
  //get the deep plate dimensions
  double deep_plate_radius, deep_plate_tolerance;  
  std::vector<double> voiMin, voiMax;
  ros::NodeHandle node_handle;

  node_handle.getParam("deep_plate_radius", deep_plate_radius);
  node_handle.getParam("deep_plate_tolerance", deep_plate_tolerance);
  node_handle.getParam("deep_plate_voi_min", voiMin);
  node_handle.getParam("deep_plate_voi_max", voiMax);
  ROS_INFO("Deep Plate: Min and max points: %f %f %f, %f %f %f", voiMin[0], voiMin[1], voiMin[2], voiMax[0], voiMax[1], voiMax[2]);

  tf::Vector3 minP, maxP;
  minP.setValue(voiMin[0], voiMin[1], voiMin[2]);
  maxP.setValue(voiMax[0], voiMax[1], voiMax[2]);

  //announce the point cloud box to be visualized in rviz
  announce_box("Deep Plate", minP, maxP, objectFrameIdName);

  //get the point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  Kinect::getInstance()->getCloud(cloud, objectFrameIdName);

  bool found = getCircularObjectPose(cloud, deep_plate_pose, deep_plate_radius, deep_plate_tolerance, minP, maxP);
  if (!found)
    return false;

  return true;
}

bool ObjectLocalizer::localizeSaltCellar(tf::Stamped<tf::Pose>& salt_cellar_pose, std::string objectFrameIdName)
{
  //get the salt cellar dimensions
  double salt_cellar_radius, salt_cellar_tolerance;  
  std::vector<double> voiMin, voiMax;
  ros::NodeHandle node_handle;

  node_handle.getParam("salt_cellar_radius", salt_cellar_radius);
  node_handle.getParam("salt_cellar_tolerance", salt_cellar_tolerance);
  node_handle.getParam("salt_cellar_voi_min", voiMin);
  node_handle.getParam("salt_cellar_voi_max", voiMax);
  ROS_INFO("Salt Cellar: Min and max points: %f %f %f, %f %f %f", voiMin[0], voiMin[1], voiMin[2], voiMax[0], voiMax[1], voiMax[2]);

  tf::Vector3 minP, maxP;
  minP.setValue(voiMin[0], voiMin[1], voiMin[2]);
  maxP.setValue(voiMax[0], voiMax[1], voiMax[2]);

  //announce the point cloud box to be visualized in rviz
  announce_box("Salt Cellar", minP, maxP, objectFrameIdName);

  //get the point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  Kinect::getInstance()->getCloud(cloud, objectFrameIdName);

  bool found = getCircularObjectPose(cloud, salt_cellar_pose, salt_cellar_radius, salt_cellar_tolerance, minP, maxP);
  if (!found)
    return false;

  return true;
}

void ObjectLocalizer::pubCloud( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud )
{
  // Try to subscribe to a given topic
  // if you can subscribe then the topic is already advertised
  // else you advertize the topic 

  if (!cloud_pub_initialized)
  {
    ros::NodeHandle node_handle;
    cloud_pub = node_handle.advertise<sensor_msgs::PointCloud2>("/popcorn_cooking_point_cloud_of_interest",0,true);
    cloud_pub_initialized=true;
  }

  sensor_msgs::PointCloud2 out; 
  out.header.frame_id = "/ikea_table";

  pcl::toROSMsg(*cloud,out);

  cloud_pub.publish(out);

  ros::Duration(0.001).sleep();

  return;
}

void ObjectLocalizer::getPointsInBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr inBox,
                                  tf::Vector3 min, tf::Vector3 max)
{
  if(cloud->points.size() <= 0)
    ROS_WARN("The received point cloud has NO points within it :(");

  Eigen::Vector4f min_pt, max_pt;
  min_pt = Eigen::Vector4f(std::min(min.x(), max.x()), std::min(min.y(), max.y()), std::min(min.z(), max.z()), 1);
  max_pt = Eigen::Vector4f(std::max(min.x(), max.x()), std::max(min.y(), max.y()), std::max(min.z(), max.z()), 1);

  boost::shared_ptr<std::vector<int> > indices( new std::vector<int> );
  pcl::getPointsInBox(*cloud, min_pt, max_pt, *indices);

  pcl::ExtractIndices<pcl::PointXYZRGB> ei;
  ei.setInputCloud(cloud);
  ei.setIndices(indices);
  ei.filter(*inBox);

  if (inBox->points.size() <= 0)
    ROS_WARN("NO point remained within the Volume of Interest ... :(");
}


bool ObjectLocalizer::getCirclesFromCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pot_cyl_cloud, 
                                       double radius_goal, double radius_tolerance,
                                       std::vector<tf::Vector3> &center,
                                       std::vector<double> &radius,
                                       std::vector<int> &numinliers,
                                       size_t  iterations,
                                       std::string circle_semantics)
{
  center.clear();
  radius.clear();
  numinliers.clear();

  center.resize(iterations);
  radius.resize(iterations);
  numinliers.resize(iterations);

  pcl::PointCloud<pcl::PointXYZRGB> act = *pot_cyl_cloud;
  pcl::PointCloud<pcl::PointXYZRGB> flip;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr circle_inliers = pot_cyl_cloud;

  // in each iteration, the inliers of the last circle found are taken out and the sacsegmentation is run again
  for (size_t k = 0; k < iterations; k++)
  {
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;

    // Optional
    seg.setOptimizeCoefficients (true);

    // Mandatory
    seg.setModelType (pcl::SACMODEL_CIRCLE2D);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.005);

    seg.setInputCloud (act.makeShared());
    seg.segment (*inliers, *coefficients);

    seg.setRadiusLimits(radius_goal - radius_tolerance, radius_goal + radius_tolerance);

    if (inliers->indices.size () == 0)
    {
      PCL_ERROR ("Could not estimate a circle model for the given dataset.");
      return false;
    }
    else
    {
      // compute the height of the center???
      double z = 0;
      for (size_t g=0; g < inliers->indices.size(); ++g)
      {
        z += act.points[inliers->indices[g]].z;
      }

      z /=  inliers->indices.size();

      center[k] = tf::Vector3(coefficients->values[0], coefficients->values[1], z);
      radius[k] = coefficients->values[2];
      numinliers[k] = inliers->indices.size();
    }

    flip = act;
        
    pcl::ExtractIndices<pcl::PointXYZRGB> inlaiers_extractor;
    inlaiers_extractor.setInputCloud(flip.makeShared());
    inlaiers_extractor.setIndices(inliers);
    inlaiers_extractor.setNegative(false);
    inlaiers_extractor.filter(*circle_inliers);
        
    pubCloud(circle_inliers);
  }

  return true;
}

bool ObjectLocalizer::getHandleRotation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, tf::Stamped<tf::Pose> &potPose,
                                     tf::Vector3 min, tf::Vector3 max)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

  double pot_handle_level_min;    
  double pot_handle_level_max;    
  double pot_handle_radius_min;   
  double pot_handle_radius_max;   
    
  ros::NodeHandle node_handle;
  node_handle.getParam("pot_handles_level_min", pot_handle_level_min);
  node_handle.getParam("pot_handles_level_max", pot_handle_level_max);
  node_handle.getParam("pot_handles_radius_min", pot_handle_radius_min);
  node_handle.getParam("pot_handles_radius_max", pot_handle_radius_max);

  tf::Vector3 newmin = tf::Vector3(min.x(), min.y(), min.z()) + tf::Vector3(0, 0, pot_handle_level_min);
  tf::Vector3 newmax = tf::Vector3(max.x(), max.y(), min.z()) + tf::Vector3(0, 0, pot_handle_level_max);
  min = newmin;
  max = newmax;

  getPointsInBox(cloud, cloud_filtered, min, max);

  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

  // we will calculate the angle of each point that we think belongs to the handle via arctan and then average over that angles
  double atansum = 0;
  int atannum = 0;
  bool pivot_set = false;

  //! TODO: this is everything but robust, if we get as pivot, orientation averaging will be wrong
  tf::Vector3 pivot(0, -100000, 0); 

  for (size_t k = 0; k < cloud_filtered->points.size(); ++k)
  {
    tf::Vector3 act(cloud_filtered->points[k].x, cloud_filtered->points[k].y, cloud_filtered->points[k].z);

    tf::Vector3 rel = act - potPose.getOrigin();
    rel.setZ(0);
    double radius = rel.length();

    if ((pot_handle_radius_min < radius) && (radius < pot_handle_radius_max))
    {
      if ((!pivot_set) || (rel.y() > pivot.y()))
      {
        pivot = rel;
        pivot_set = true;
      }
    }
  }

  for (size_t k = 0; k < cloud_filtered->points.size(); ++k)
  {
    tf::Vector3 act(cloud_filtered->points[k].x, cloud_filtered->points[k].y, cloud_filtered->points[k].z);

    tf::Vector3 rel = act - potPose.getOrigin();
    rel.setZ(0);
    double radius = rel.length();

    if ((pot_handle_radius_min < radius) && (radius < pot_handle_radius_max))
    {
      if (rel.x() != 0.0)
      {
        inliers->indices.push_back(k);

        double at;
        if ((pivot - rel).length() < 0.1)
        {
          at = atan2(rel.y(), rel.x());
        }
        else
        {
          at = atan2(-rel.y(), -rel.x());
        }
        
        atansum += at;
        atannum += 1;
      }
    }
  }

  ROS_INFO("HANDLE INLIERS %d out of %zu total points", atannum, cloud_filtered->points.size());

  if (atannum < 20)
  {
    ROS_ERROR("did not get enough handle inliers");
    return false;
  }

  atansum /= atannum;

  potPose.setRotation( tf::Quaternion( tf::Vector3(0, 0, 1), atansum ));
    
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr handles(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::ExtractIndices<pcl::PointXYZRGB> ei;
  ei.setInputCloud(cloud_filtered);
  ei.setIndices(inliers);
  ei.setNegative(false);
  ei.filter(*handles);

  pubCloud(handles);

  return true;
}

bool ObjectLocalizer::getCenter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, tf::Vector3 &center)
{
  center = tf::Vector3(0, 0, 0);

  if (cloud->points.size() < 1)
    return false;

  for (size_t k = 0; k < cloud->points.size(); ++k)
  {
    tf::Vector3 act(cloud->points[k].x, cloud->points[k].y, cloud->points[k].z);
    center += act;
  }

  center *= (1 / (double) cloud->points.size());

  return true;
}

bool ObjectLocalizer::getLidPose(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, tf::Stamped<tf::Pose> &lid,
                              double radius_goal, tf::Vector3 min, tf::Vector3 max)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr inBox (new pcl::PointCloud<pcl::PointXYZRGB>);
  getPointsInBox(cloud, inBox, min, max);

  // announce_box("lid - VOI", min, max);
  pubCloud(inBox);

  std::vector<tf::Vector3> center;
  std::vector<double> radius;
  std::vector<int> numinliers;

  double lid_radius_expected;
  double lid_radius_tolerance;

  ros::NodeHandle node_handle;
  node_handle.getParam("lid_radius_expected", lid_radius_expected);
  node_handle.getParam("lid_radius_tolerance", lid_radius_tolerance);

  int iterations = 1;
  std::string circle_semantics("lid detection");
  getCirclesFromCloud(inBox, lid_radius_expected, lid_radius_tolerance, center, radius, numinliers, iterations, circle_semantics);

  // sanity check, we get will usually see a lot more points
  if ((numinliers[0] < 50) ||                                    // too few inliers
      (fabs(radius[0] - radius_goal) > lid_radius_tolerance) ||  // radius too different than the expected radius
      (center[0].x() < min.x()) ||                               // center 
      (center[0].y() < min.y()) ||                               // outside of
      (center[0].x() > max.x()) ||                               // Volume of
      (center[0].y() > max.y()))                                 // Interest
  {
    ROS_ERROR("Circle with %d inliers, radius %f and center at %f %f is unacceptable :( ",
              numinliers[0], radius[0], center[0].x(), center[0].y());
    return false;
  }
  else
  {
    ROS_WARN("DETECTED Circle with %d inliers, radius %f and center at %f %f :)",
             numinliers[0], radius[0], center[0].x(), center[0].y());
  }

  lid.setOrigin(center[0]);
  lid.setRotation(tf::Quaternion(0, 0, 0, 1));
       
  return true;
}

bool ObjectLocalizer::getPotPoseViaLid( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, tf::Stamped<tf::Pose> &potPose,
                                     tf::Vector3 min, tf::Vector3 max)
{
  // announce_box("Pot via Lid", min, max);

  // Note: The idea!
  // compute Pot's center via Lid or via top edges
  // compute Pot's orientation via Pot Handles
  
  double table_height = min.z();

  // some values for our magical pot, todo: move to parameters, xml or something
  double pot_body_radius;     // = 0.20 
  double lid_level_min;       // = 0.09
  double lid_level_max;       // = 0.12

  ros::NodeHandle node_handle;
  node_handle.getParam("pot_body_radius", pot_body_radius);
  node_handle.getParam("lid_level_min", lid_level_min);
  node_handle.getParam("lid_level_max", lid_level_max);

  tf::Vector3 minLid = tf::Vector3(min.x(), min.y(), min.z()) + tf::Vector3(0, 0, lid_level_min);
  tf::Vector3 maxLid = tf::Vector3(max.x(), max.y(), min.z()) + tf::Vector3(0, 0, lid_level_max);

  //! find the pot center
  tf::Vector3 center;
  tf::Stamped<tf::Pose> lidPose;

  if (!getLidPose(cloud, lidPose, pot_body_radius, minLid, maxLid))
  {
    ROS_ERROR("Lid could NOT be localized!");
    return false;
  }
  else
  {
    center = lidPose.getOrigin();
  }

  //! find the handles on the pot side
  center.setZ(table_height);

  // the pose of the POT is the center of the lid!
  potPose.setOrigin(center);
    
  /*
  //skip the handle rotation method
  if (!getHandleRotation(cloud, potPose, min, max))
  {
    ROS_ERROR("Could not get Hanndle Orientation");
    return false;
  }
  */

  return true;
}

bool ObjectLocalizer::getCircularObjectPose(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, 
                                         tf::Stamped<tf::Pose> &object_pose,
                                         double radius_goal, double radius_tolerance,
                                         tf::Vector3 min, tf::Vector3 max)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr inBox (new pcl::PointCloud<pcl::PointXYZRGB>);
  getPointsInBox(cloud, inBox, min, max);

  pubCloud(inBox);

  std::vector<tf::Vector3> center;
  std::vector<double> radius;
  std::vector<int> numinliers;

  // for now we assume we see only one circle fitting and this will be the one
  // todo: get multiple hyptheses and preserve them
  int iterations = 10;
  std::string circle_semantics("circular object");
  getCirclesFromCloud(inBox, radius_goal, radius_tolerance, center, radius, numinliers, iterations, circle_semantics);

  int center_size = center.size();
  for (int k = 0; k < center_size; ++k)
  {
    ROS_INFO("NUM_INLIERS : %i", numinliers[k]);

    if (fabs(radius[k] - radius_goal) > radius_tolerance)
    {
      ROS_ERROR("Detected Circle outside radius bounds! %f outside of %f +- %f", radius[k], radius_goal, radius_tolerance);
      continue;
//      return false;
    }
    if ((center[k].x() < min.x()) || 
        (center[k].y() < min.y()) || 
        (center[k].x() > max.x()) || 
        (center[k].y() > max.y()))
    {
      ROS_ERROR("Detected Circle center outside bounding box!");
      continue;
//      return false;
    }

    // sanity check, we get will usually see a lot more points
    if (numinliers[0] < 80)
      continue;
//      return false;

    center[k].setZ( min.z() );

    object_pose.setOrigin(center[k]);
    object_pose.setRotation(tf::Quaternion(0, 0, 0, 1));
    object_pose.frame_id_ = "/map";

    ROS_WARN("Detected Circle with radius %f, when the radius goal is %f +- %f", radius[k], radius_goal, radius_tolerance);

    return true;
  }

  return false;
}


//vars for announce box method
ros::Publisher perc_announcer; // perception announcer, sends a marker whenever something is perceived
bool perc_init = false;
int marker_id = 1;

void ObjectLocalizer::announce_box(const std::string text, tf::Vector3 min, tf::Vector3 max, std::string objectFrameIdName)
{ 
  
  if (!perc_init)
  {
    perc_init = true;
    ros::NodeHandle node_handle;
    perc_announcer =  node_handle.advertise<visualization_msgs::Marker>( "/popcorn_cooking_volume_of_interest", 0 , true);
  }

  visualization_msgs::Marker marker;
  marker.header.frame_id = objectFrameIdName;
  marker.header.stamp = ros::Time::now();
  marker.ns = "popcorn_perception_volume_of_interest";
  marker.id =  marker_id++;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;
  marker.pose.position.x = 0.5 * (max.x() + min.x());
  marker.pose.position.y = 0.5 * (max.y() + min.y());
  marker.pose.position.z = 0.5 * (max.z() + min.z());
  marker.scale.x = max.x() - min.x();
  marker.scale.y = max.y() - min.y();
  marker.scale.z = max.z() - min.z();
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 0.4;
  marker.color.a = 0.2;
  marker.lifetime = ros::Duration(10.0);

  perc_announcer.publish(marker);
  ros::Duration(0.001).sleep();

  marker.id =  marker_id++;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.text = text;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 0.4;
  marker.color.a = 0.4;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.lifetime = ros::Duration(30.0);

  perc_announcer.publish(marker);
  ros::Duration(0.001).sleep();
}

