#include "local_goal_creator/local_goal_creator.h"

LocalGoalCreator::LocalGoalCreator():private_nh_("~")
{
    private_nh_.getParam("hz_", hz_);
    private_nh_.getParam("goal_index_", goal_index_);
    private_nh_.getParam("local_goal_dist_", local_goal_dist_);

    global_path_sub_ = nh_.subscribe("/global_path", 1, &LocalGoalCreator::global_path_callback, this);
    current_pose_sub_ = nh_.subscribe("/current_pose", 1, &LocalGoalCreator::current_pose_callback, this);

    local_goal_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/local_goal", 1);
}

void LocalGoalCreator::global_path_callback(const nav_msgs::Path::ConstPtr &msg)
{
    global_path_ = *msg;
    local_goal_.point.x = global_path_.poses[goal_index_].pose.position.x;
    local_goal_.point.y = global_path_.poses[goal_index_].pose.position.y;
    is_global_path_checker_ = true;
}

void LocalGoalCreator::current_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_pose_ = *msg;
    is_current_pose_checker_ = true;
}

void LocalGoalCreator::select_local_goal()
{
    double dx = current_pose_.pose.position.x - global_path_.poses[goal_index_].pose.position.x;
    double dy = current_pose_.pose.position.y - global_path_.poses[goal_index_].pose.position.y;
    double distance = hypot(dx, dy);

    if(distance < local_goal_dist_)
    {
        goal_index_ += 3;

        if(goal_index_ < global_path_.poses.size())
        {
            local_goal_.point.x = global_path_.poses[goal_index_].pose.position.x;
            local_goal_.point.y = global_path_.poses[goal_index_].pose.position.y;
        }
        else
        {
            goal_index_ = global_path_.poses.size() -1;
            local_goal_.point.x = global_path_.poses[goal_index_].pose.position.x;
            local_goal_.point.y = global_path_.poses[goal_index_].pose.position.y;
        }
    }
   // local_goal_.pose.orientation.w = 1;
}

void LocalGoalCreator::process()
{
    ros::Rate loop_rate(hz_);
    while(ros::ok()) {
        if((is_global_path_checker_ = true) && (is_current_pose_checker_ = true))
        {
            select_local_goal();
            local_goal_.header.frame_id = "map";
            local_goal_pub_.publish(local_goal_);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "local_goal_creator");
    LocalGoalCreator localgoalcreator;
    localgoalcreator.process();

    return 0;
}
