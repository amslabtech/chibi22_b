#ifndef LOCAL_GOAL_CREATOR_H
#define LOCAL_GOAL_CREATOR_H

#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"

class LocalGoalCreator
{
    public:
        LocalGoalCreator();
        void process();
    private:
        void global_path_callback(const nav_msgs::Path::ConstPtr &msg);
        void current_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void select_local_goal();

        int hz_;
        int goal_index_;
        double local_goal_dist_;
        bool is_global_path_checker_ = false;
        bool is_current_pose_checker_ = false;

        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        ros::Subscriber global_path_sub_;
        ros::Subscriber current_pose_sub_;
        ros::Publisher local_goal_pub_;

        nav_msgs::Path global_path_;
        geometry_msgs::PoseStamped current_pose_;
        geometry_msgs::PointStamped local_goal_;
};

#endif
