#include <popcorn_perception/Geometry.h>

tf::TransformListener *Geometry::listener_=0;

void Geometry::init()
{
    if (!listener_)
        listener_ = new tf::TransformListener();
}

tf::Stamped<tf::Pose> Geometry::getPoseIn(const char target_frame[], tf::Stamped<tf::Pose>src)
{

    if (src.frame_id_ == "NO_ID_STAMPED_DEFAULT_CONSTRUCTION")
    {
        ROS_ERROR("Frame not in TF: %s", src.frame_id_.c_str());
        tf::Stamped<tf::Pose> pose;
        return pose;
    }

    if (!listener_)
        listener_ = new tf::TransformListener(ros::Duration(30));

    tf::Stamped<tf::Pose> transform;
    //this shouldnt be here TODO
    src.stamp_ = ros::Time(0);

    listener_->waitForTransform(src.frame_id_, target_frame,
                                ros::Time(0), ros::Duration(30.0));
    bool transformOk = false;
    while (!transformOk)
    {
        try
        {
            transformOk = true;
            listener_->transformPose(target_frame, src, transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("getPoseIn %s",ex.what());
            // dirty:
            src.stamp_ = ros::Time(0);
            transformOk = false;
        }
        ros::spinOnce();
    }
    return transform;
}

tf::Stamped<tf::Pose> Geometry::getPose(const char target_frame[],const char lookup_frame[], ros::Time tm)
{

    init();
    //tf::TransformListener listener;
    ros::Rate rate(100.0);

    tf::StampedTransform transform;
    bool transformOk = false;
    bool had_to_retry = false;

    //listener_->waitForTransform(target_frame, lookup_frame, tm, ros::Duration(0.5));
    while (ros::ok() && (!transformOk))
    {
        transformOk = true;
        try
        {
            listener_->lookupTransform(target_frame, lookup_frame,tm, transform);
        }
        catch (tf::TransformException ex)
        {
            std::string what = ex.what();
            what.resize(100);
            ROS_INFO("getPose: tf::TransformException ex.what()='%s', will retry",what.c_str());
            transformOk = false;
            had_to_retry = true;
            //listener_->waitForTransform(target_frame, lookup_frame, ros::Time(0), ros::Duration(0.5));
        }
        if (!transformOk)
            rate.sleep();
    }

    if (had_to_retry)
        ROS_INFO("Retry sucessful");

    tf::Stamped<tf::Pose> ret;
    ret.frame_id_ = transform.frame_id_;
    ret.stamp_ = transform.stamp_;
    ret.setOrigin(transform.getOrigin());
    ret.setRotation(transform.getRotation());

    return ret;
}

tf::Stamped<tf::Pose> Geometry::rotateAroundPose(tf::Stamped<tf::Pose> toolPose, tf::Stamped<tf::Pose> pivot, double r_x, double r_y, double r_z)
{
    tf::Transform curr = toolPose;
    tf::Transform pivo = pivot;

    curr = pivo.inverseTimes(curr);

    tf::Quaternion qa;
    qa.setEulerZYX(r_z,r_y,r_x);

    tf::Transform rot;
    rot.setOrigin(tf::Vector3(0,0,0));
    rot.setRotation(qa);
    curr = rot * curr;
    curr = pivo * curr;

    tf::Stamped<tf::Pose> act;
    act.frame_id_ = toolPose.frame_id_;
    act.setOrigin(curr.getOrigin());
    act.setRotation(curr.getRotation());

    return act;
}


tf::Stamped<tf::Pose> Geometry::rotateAroundPose(tf::Stamped<tf::Pose> toolPose, tf::Stamped<tf::Pose> pivot, tf::Quaternion qa)
{
    tf::Transform curr = toolPose;
    tf::Transform pivo = pivot;

    curr = pivo.inverseTimes(curr);

    tf::Transform rot;
    rot.setOrigin(tf::Vector3(0,0,0));
    rot.setRotation(qa);
    curr = rot * curr;
    curr = pivo * curr;

    tf::Stamped<tf::Pose> act;
    act.frame_id_ = toolPose.frame_id_;
    act.setOrigin(curr.getOrigin());
    act.setRotation(curr.getRotation());

    return act;
}


tf::Stamped<tf::Pose> Geometry::approach(tf::Stamped<tf::Pose> toolPose, double dist)
{
    tf::Stamped<tf::Pose> appr;
    appr.setOrigin(tf::Vector3(- dist, 0, 0));
    appr.setRotation(tf::Quaternion(0,0,0,1));

    tf::Stamped<tf::Pose> ret;
    ret = toolPose;
    ret *= appr;
    return ret;
}

tf::Stamped<tf::Pose> Geometry::high(tf::Stamped<tf::Pose> toolPose, double dist)
{
    tf::Stamped<tf::Pose> appr = toolPose;
    appr.getOrigin()+=tf::Vector3(0, 0, dist);
    return appr;
}

tf::Stamped<tf::Pose> Geometry::make_pose(double x, double y, double z, double ox, double oy, double oz, double ow, const char target_frame[])
{
    tf::Stamped<tf::Pose> toolTargetPose;

    toolTargetPose.frame_id_ = target_frame;
    toolTargetPose.stamp_ = ros::Time(0);
    toolTargetPose.setOrigin(tf::Vector3( x,y,z));
    toolTargetPose.setRotation(tf::Quaternion(ox,oy,oz,ow));

    return toolTargetPose;
}

tf::Stamped<tf::Pose> Geometry::make_pose(const tf::Transform &trans, const char target_frame[])
{
    tf::Stamped<tf::Pose> toolTargetPose;
    toolTargetPose.frame_id_ = target_frame;
    toolTargetPose.stamp_ = ros::Time(0);
    toolTargetPose.setOrigin(trans.getOrigin());
    toolTargetPose.setRotation(trans.getRotation());

    return toolTargetPose;
}

