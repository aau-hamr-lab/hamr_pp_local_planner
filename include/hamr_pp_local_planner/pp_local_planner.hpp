#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>

namespace hamr_pp_local_planner
{
class PurePursuitPlanner : public nav_core::BaseLocalPlanner
{
public:
  PurePursuitPlanner();

  /**
  * @brief  Given the current position, orientation, and velocity of the robot, compute velocity commands to send to
  * the base
  * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
  * @return True if a valid velocity command was found, false otherwise
  */
  bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);

  /**
    * @brief  Check if the goal pose has been achieved by the local planner
    * @return True if achieved, false otherwise
    */
  bool isGoalReached();

  /**
    * @brief  Set the plan that the local planner is following
    * @param plan The plan to pass to the local planner
    * @return True if the plan was updated successfully, false otherwise
    */
  bool setPlan(const std::vector<geometry_msgs::PoseStamped> &plan);

  /**
    * @brief  Constructs the local planner
    * @param name The name to give this instance of the local planner
    * @param tf A pointer to a transform listener
    * @param costmap_ros The cost map to use for assigning costs to local plans
    */
  void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS *costmap_ros);

  bool isInitialized()
  {
    return initialized_;
  }

private:
  bool initialized_;

  ros::Publisher g_plan_pub_, l_plan_pub_;

  tf::TransformListener* tf_;
  costmap_2d::Costmap2DROS *costmap_ros_;
  base_local_planner::LocalPlannerUtil planner_util_;
  tf::Stamped<tf::Pose> current_pose_;
  base_local_planner::OdometryHelperRos odom_helper_;
  std::string odom_topic_;
};

}  // namespace hamr_pp_local_planner