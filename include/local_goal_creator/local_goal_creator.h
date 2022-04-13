#ifndef LOCAL_GOAL_CREATOR_H
#define LOCAL_GOAL_CREATOR_H

#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"

class LocalGoalCreator
{
    public:
        local_goal_creator();
        void process();
    private:
        void global_path_callback(const nav_msgs::Path::ConstPtr &msg);
        void current_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void select_local_goal();

        int hz_;
        int goal_index_;
        double local_goal_dist_;
        bool is_global_path_checker = false;
        bool is_current_pose_checker = false;

        ros::NodeHandle nh;
        ros::NodeHandle private_nh;
        ros::Subscriber global_path_sub;
        ros::Subscriber current_pose_sub;
        ros::Publisher local_goal_pub;

        nav_msgs::Path global_path;
        geometry_msgs::PoseStamped current_pose;
        geometry_msgs::PoseStamped local_goal;
};

#endif
