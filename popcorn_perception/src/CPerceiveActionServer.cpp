
#include <popcorn_perception/CPerceiveActionServer.h>


CPerceiveActionServer::CPerceiveActionServer(std::string name) :
    as_(nh_, name, boost::bind(&CPerceiveActionServer::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  CPerceiveActionServer::~CPerceiveActionServer(void)
  {
  }

  void CPerceiveActionServer::executeCB(const popcorn_perception::PerceiveGoalConstPtr &goal)
  {
    // helper variables
    bool success = true;

    //clear the result
    result_.object_pose.clear();

    // publish info to the console for the user
    ROS_INFO("%s: Executing, perceiving the object: %s, with the frame id: %s", action_name_.c_str(), goal->object_type.c_str(), goal->object_frame_id_name.c_str());
    
    // check that preempt has not been requested by the client
    if (as_.isPreemptRequested() || !ros::ok())
    {
        ROS_INFO("%s: Preempted.", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
    }
    else 
    {
        // start executing the action
        tf::Stamped<tf::Pose> object_pose;
        ObjectLocalizer* objectLocalizer = new ObjectLocalizer();
        
        if (goal->object_type == "pot")
        {
           success = objectLocalizer->localizePot(object_pose, goal->object_frame_id_name.c_str());
        }
         else
         if (goal->object_type == "lid")
         {
           success = objectLocalizer->localizeLid(object_pose, goal->object_frame_id_name.c_str());
         }
        else
         if (goal->object_type == "small_bowl")
         {
           success = objectLocalizer->localizeSmallBowl(object_pose, goal->object_frame_id_name.c_str());
         }
         else
          if (goal->object_type == "deep_plate")
          {
             success = objectLocalizer->localizeDeepPlate(object_pose, goal->object_frame_id_name.c_str());
          }
          else
           if (goal->object_type == "salt_cellar")
           {
              success = objectLocalizer->localizeSaltCellar(object_pose, goal->object_frame_id_name.c_str());
           }
           else
           {
             success = false;
           }

        // create the response
        tf::Vector3 t = object_pose.getOrigin();
        result_.object_pose.push_back(t.x());
        result_.object_pose.push_back(t.y());
    }

    if(success)
    {
      feedback_.object_name = "Object: " + goal->object_type + " detected! :)";
      
      // set the action state to succeeded
      as_.setSucceeded(result_);
      ROS_INFO("%s: Succeeded. ", action_name_.c_str());
      ROS_INFO("Result... %f %f", result_.object_pose[0], result_.object_pose[1]);
    }
    else 
    {
      feedback_.object_name = "Object: " + goal->object_type + " not detected. :(";
   
      // set the action state to aborted
      as_.setAborted(result_);
      ROS_INFO("%s: Aborted. ", action_name_.c_str());
    }

    // publish the feedback
    as_.publishFeedback(feedback_);
    ROS_INFO("Feedback... %s.", feedback_.object_name.c_str());

  }
