#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "nunavigation/guide.hpp"


using MoveBaseClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

move_base_msgs::MoveBaseGoal makeGoal(math::vecf<2> goal_point, 
                                      std::string const& goal_frame)
{
  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = goal_frame;
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = goal_point[0];
  goal.target_pose.pose.position.y = goal_point[1];
  goal.target_pose.pose.orientation.w = 1;

  return goal;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_explorer");

  Guide guide{"map", "slam/pose"};
  // ros::spin();
  // tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
  // wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  ros::Duration(1).sleep();
  ros::spinOnce();

  while(ros::ok())
  {
    std::pair<math::vecf<2>, std::string> goal = guide.nextGoal();

    auto goal_point = goal.first; 
    ROS_INFO_STREAM("goal: " << goal_point.transpose());

    int inf = std::numeric_limits<int>::infinity();
    if(goal_point[0] == inf && goal_point[1] == inf)
      break;

    auto goal_msg = makeGoal(goal_point, goal.second);
    ac.sendGoal(goal_msg);
    
    ac.waitForResult();
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Hooray, the base moved towards the goal successfully");
    else
      ROS_INFO("The base failed to move forward for some reason");
    
    ros::Duration(1).sleep();
    ros::spinOnce();
  }

  return 0;
}