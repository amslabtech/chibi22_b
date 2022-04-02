#include "local_goal_creator/local_goal_creator.h"

LocalGoalCreator::local_goal_creator():private_nh("~")
{
    private_nh.getParam("hz", hz);
    private_nh.getParam("local_goal_dist", local_goal_dist);
    private_nh.getParam("lap_num", lap_num);

    global_path_sub = nh.subscribe("/global_path", 1, &LocalGoalCreator::global_path_callback, this);
    estimated_pose_sub = nh.subscribe("/estimated_pose", 10, &LocalGoalCreator::estimated_pose_callback, this);

    local_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/local_goal", 10);
}

void LocalGoalCreator::global_path_callback(const nav_nsgs::Path::ConstPtr &msg)
{
    global_path = *msg;
    global_path_get_flag = true;
}

void LocalGoalCreator::estimated_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    estimated_pose = *msg;
    estimated_pose_get_flag = true;
    if(global_path_get_flag) {
        select_local_goal();
    }
}

void LocalGoalCreator::select_local_goal()
{
    double dist = sqrt(pow(estimated_pose.pose.position.x - global_path.poses[goal_index].pose.position.x, 2) + pow(estimated_pose.pose.position.y - global_path.poses[goal_index].pose.position.y, 2));

    while((dist < local_goal_dist) && ((goal_index + 1) < int(global_path.poses.size()))) {
        dist += 0.05;
        goal_index += 1;
    }
    local_goal = global_path.poses[goal_index];
    local_goal.header.frame_id = "map";
    local_goal.header.stamp = ros::Time::now();
    local_goal.pose.orientation.w = 1;

    if((goal_index == int(global_path.poses.size()) - 1) && (lap_count < lap_num)) {
        goal_index = 0;
        lap_count += 1;
    }
}

void LocalGoalCreator::process()
{
    ros::Rate loop_rate(hz);
    while(ros::ok()) {
        if(global_path_get_flag && estimated_pose_get_flag) {
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
