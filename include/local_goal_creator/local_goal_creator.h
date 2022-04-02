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
        void estimated_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void select_local_goal();

        int hz;
        double local_goal_dist;
        int lap_num;
        int goal_index = 0;
        int lap_count = 1;
        bool global_path_get_flag = false;
        bool estimated_pose_get_flag = false;

        ros::NodeHandle nh;
        ros::NodeHandle private_nh;
        ros::Subscriber global_path_sub;
        ros::Subscriber estimated_pose_sub;
        ros::Publisher local_goal_pub;

        nav_msgs::Path global_path;
        geometry_msgs::PoseStamped estimated_pose;
        geometry_msgs::PoseStamped local_goal;
};

#endif
