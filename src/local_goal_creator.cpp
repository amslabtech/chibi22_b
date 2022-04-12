#include "local_goal_creator/local_goal_creator.h"

LocalGoalCreator::local_goal_creator():private_nh("~")
{
    private_nh.getParam("hz_", hz_);
    private_nh.getParam("local_goal_dist_", local_goal_dist_);
    private_nh.getParam("goal_index_", goal_index_);

    global_path_sub = nh.subscribe("/global_path", 1, &LocalGoalCreator::global_path_callback, this);
    current_pose_sub = nh.subscribe("/current_pose", 10, &LocalGoalCreator::current_pose_callback, this);

    local_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/local_goal", 10);
}

void LocalGoalCreator::global_path_callback(const nav_nsgs::Path::ConstPtr &msg)
{
    global_path = *msg;
    local_goal = global_path.poses[goal_index_];
    is_global_path_checker = true;
}

void LocalGoalCreator::current_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_pose = *msg;
    is_current_pose_checker = true;
}

void LocalGoalCreator::select_local_goal()
{
    double dx = current_pose.pose.position.x - global_path.poses[goal_index_].pose.position.x;
    double dy = current_pose.pose.position.y - global_path.poses[goal_index_].pose.position.y;
    double distance = hypot(dx, dy);

    if(distance < local_goal_dist_)
    {
        goal_index_ += 3;

        if(goal_index_ < global_path.poses.size())
        {
            local_goal = global_path.poses[goal_index_];
        }
        else
        {
            goal_index_ = global_path.poses.size() -1;
            local_goal = global_path.poses[goal_index_];
        }
    }
    local_goal.pose.orientation.w = 1;
}

void LocalGoalCreator::process()
{
    ros::Rate loop_rate(hz_);
    while(ros::ok()) {
        if((is_global_path_checker = true) && (is_current_pose_checker = true))
        {
            select_local_goal();
            local_goal.header.frame_id = "map";
            local_goal_pub.publish(local_goal);
        }
        ros::spinOnce();
        loop_late.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "local_goal_creator");
    LocalGoalCreator localgoalcreator;
    localgoalcreator.process();

    return 0;
}
