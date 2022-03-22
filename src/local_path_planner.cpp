#include "local_path_planner/local_path_planner.h"

LocalPathPlanner::LocalPathPlanner():private_nh_("~"), nh_("")
{
    private_nh_.getParam("hz", hz);
    private_nh_.getParam("max_vel", max_vel_);
    private_nh_.getParam("min_vel", min_vel_);
    private_nh_.getParam("max_yawrate", max_yawrate_);
    private_nh_.getParam("max_accel", max_accel_);
    private_nh_.getParam("max_dyawrate", max_dyawrate_);
    private_nh_.getParam("vel_reso", vel_reso_);
    private_nh_.getParam("yawrate_reso", yawrate_reso_);
    private_nh_.getParam("dt", dt_);
    private_nh_.getParam("predict_time", predict_time_);
    private_nh_.getParam("heading_cost_gain", heading_cost_gain_);
    private_nh_.getParam("dist_cost_gain", dist_cost_gain_);
    private_nh_.getParam("vel_cost_gain", vel_cost_gain_);

    sub_local_map_ = nh_.subscribe("/local_map", 10, &LocalPathPlanner::local_map_callback, this);
    sub_local_goal_ = nh_.subscribe("/local_goal", 10, &LocalPathPlanner::local_goal_callback, this);
    sub_pose_        = nh_.subscribe("/roomba/pose", 10, &LocalPathPlanner::pose_callback, this);

    pub_cmd_vel_ = nh_.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control", 1);
}

// local_mapのコールバック関数
void LocalPathPlanner::local_map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    local_map_ = *msg;
}

// local_goalのコールバック関数
void LocalPathPlanner::local_goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    local_goal_ = *msg;
}

// poseのコールバック関数
void LocalPathPlanner::pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose_ = *msg;
}

void LocalPathPlanner::roomba_control(float velocity, float yawrate)
{
    cmd_vel_.mode = 11; // 任意の動きを実行するモード
    cmd_vel_.cntl.linear.x = velocity;
    cmd_vel_.cntl.angular.z = yawrate;
    pub_cmd_vel_.publish(cmd_vel_);
}

void LocalPathPlanner::move(float *x, float u, float dt)
{

}

void LocalPathPlanner::calc_dynamic_window(float dw[])
{
    // 車両モデルによるWindow
    float Vs[] = {min_vel_, max_vel_, -max_yawrate_, max_yawrate_};

    // 運動モデルによるWindow
    float Vd[] = {roomba_.velocity - max_accel_*dt_,
                       roomba_.velocity + max_accel_*dt_,
                       roomba_.yawrate - max_dyawrate_*dt_,
                       roomba_.yawrate - max_dyawrate_*dt_};

    // 最終的なDynamic Window
    dw[] = {std::max(Vs[0], Vd[0]),
               std::min(Vs[1], Vd[1]),
               std::max(Vs[2], Vd[2]),
               std::min(Vs[3], Vd[3])};
}

void LocalPathPlanner::calc_trajectory()
{

}

void LocalPathPlanner::calc_final_input()
{

}

float  LocalPathPlanner::calc_evaluation()
{
    float heading_score = weight_heading_ * calc_heading_eval();
    float distance_score = weight_dist_ * calc_dist_eval();
    float velocity_score  = weight_vel_ * calc_vel_eval();
    float total_score       = heading_score + distance_score + velocity_score;

    return total_score;
}

// headingの評価関数を計算する
float LocalPathPlanner::calc_heading_eval()
{
    // ロボットの方位
    float theta = roomba_.yaw;
    // ゴールの方位
    // +の向きをroombaに合わせる
    float goal_theta = -std::atan2(local_goal_.pose.position.y - roomba_.y, local_goal_.pose.position.x - roomba_.x);
    // ゴールまでの方位差分
    float target_theta = goal_theta - theta;
    // headingの評価値
    float heading_eval = M_PI - std::abs(target_theta);
    return heading_eval;
}

float LocalPathPlanner::calc_dist_eval()
{

}

float LocalPathPlanner::calc_vel_eval()
{
    return roomba_.velocity;
}

void LocalPathPlanner::dwa_control()
{

}

// main文
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "local_path_planner"); // ノードの初期化
    LocalPathPlanner local_path_planner;
    local_path_planner.process();

    return 0;
}