#include <pp_local_planner.hpp>
#include <nav_msgs/Path.h>

namespace hamr_pp_local_planner
{
bool PurePursuitPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel){
  cmd_vel.linear.x = 0.1;
  cmd_vel.linear.y = 0.0;
  cmd_vel.linear.z = 0.0;

  cmd_vel.angular.x = 0.0;
  cmd_vel.angular.y = 0.0;
  cmd_vel.angular.z = 0.0;

  return true;
};

bool PurePursuitPlanner::isGoalReached(){
  return false;
};

bool PurePursuitPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan){
  return true;
};

void PurePursuitPlanner::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS *costmap_ros){
  if ( !isInitialized() )
  {
    ros::NodeHandle private_nh("~/" + name);
    g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
    l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    costmap_ros_->getRobotPose(current_pose_);

    // make sure to update the costmap we'll use for this cycle
    costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap();

    planner_util_.initialize(tf, costmap, costmap_ros_->getGlobalFrameID());

    if (private_nh.getParam("odom_topic", odom_topic_))
    {
      odom_helper_.setOdomTopic(odom_topic_);
    }

    initialized_ = true;
  }
  else
  {
    ROS_WARN("This planner has already been initialized, doing nothing.");
  }
};

PurePursuitPlanner::PurePursuitPlanner() : initialized_(false), odom_helper_("odom")
{
}

}  // namespace hamr_pp_local_planner