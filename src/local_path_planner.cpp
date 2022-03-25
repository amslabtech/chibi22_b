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

    sub_local_map_ = nh_.subscribe("/local_map/local_map", 10, &LocalPathPlanner::local_map_callback, this);
    sub_local_goal_ = nh_.subscribe("/local_goal", 10, &LocalPathPlanner::local_goal_callback, this);
    sub_pose_        = nh_.subscribe("/roomba/pose", 10, &LocalPathPlanner::pose_callback, this);
    sub_ob_poses_ = nh_.subscribe("/local_map/obstacle", 10, &LocalPathPlanner::ob_poses_callback, this);

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

// ob_posesのコールバック
void LocalPathPlanner::ob_poses_callback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    ob_poses_ = *msgs;
}

// Roombaの制御入力
void LocalPathPlanner::roomba_control(float velocity, float yawrate)
{
    cmd_vel_.mode = 11; // 任意の動きを実行するモード
    cmd_vel_.cntl.linear.x = velocity;
    cmd_vel_.cntl.angular.z = yawrate;
    pub_cmd_vel_.publish(cmd_vel_);
}

// 予測軌跡作成時における仮想ロボットの移動
void LocalPathPlanner::move(State& state, float velocity, float yawrate)
{
    state.yaw += yawrate * dt_;
    state.x += velocity * std::cos(state.yaw) * dt_;
    state.y += velocity * std::sin(state.yaw) * dt_;
    state.velocity = velocity;
    state.yawrate = yawrate;
}

// Dynamic Windowを計算
void LocalPathPlanner::calc_dynamic_window()
{
    // 車両モデルによるWindow
    float Vs[] = {min_vel_, max_vel_, -max_yawrate_, max_yawrate_};

    // 運動モデルによるWindow
    float Vd[] = {roomba_.velocity - max_accel_*dt_,
                       roomba_.velocity + max_accel_*dt_,
                       roomba_.yawrate - max_dyawrate_*dt_,
                       roomba_.yawrate - max_dyawrate_*dt_};

    // 最終的なDynamic Window
    dw_.min_vel = std::max(Vs[0], Vd[0]),
    dw_.max_vel= std::min(Vs[1], Vd[1]),
    dw_.min_yawrate = std::max(Vs[2], Vd[2]),
    dw_.max_yawrate = std::min(Vs[3], Vd[3]);
}

// 予測軌跡の作成
std::vector<State> LocalPathPlanner::calc_trajectory(float velocity, float yawrate)
{
    State state = {0.0, 0.0, 0.0, 0.0, 0.0}; // 軌跡作成用の仮想ロボット
    std::vector<State> trajectory; // 軌跡格納用の動的配列

    // 軌跡を格納
    for(double t=0.0; t<=predict_time_; t+=dt_)
    {
        move(state, velocity, yawrate);
        trajectory.push_back(state);
    }

    return trajectory;
}

// 最適な制御入力を計算 {velocity, yawrate}
std::vector<float> LocalPathPlanner::calc_final_input()
{
    std::vector<float> input{0.0, 0.0}; // {velocity, yawrate}
    float max_score = 0.0;
    
    for(float velocity=dw_.min_vel; velocity<=dw_.max_vel; velocity+=vel_reso_)
    {
        for(float yawrate=dw_.min_yawrate; yawrate<=dw_.max_yawrate; yawrate+=yawrate_reso_)
        {
            std::vector<State> trajectory = calc_trajectory(velocity, yawrate); // 予測軌跡の生成
            float score = calc_evaluation(trajectory); // 予測軌跡に対する評価値の計算

            // 最大値の更新
            if(max_score < score)
            {
                max_score = score;
                input[0] = velocity;
                input[1] = yawrate;
            }
        }
    }

    return input;
}

// 評価関数を計算
float  LocalPathPlanner::calc_evaluation(std::vector<State> traj)
{
    float heading_score = weight_heading_ * calc_heading_eval(traj);
    float distance_score = weight_dist_ * calc_dist_eval(traj);
    float velocity_score  = weight_vel_ * calc_vel_eval(traj);
    float total_score       = heading_score + distance_score + velocity_score;

    return total_score;
}

// headingの評価関数を計算
float LocalPathPlanner::calc_heading_eval(std::vector<State> traj)
{
    // 最終時刻のロボットの方位
    float theta = traj.back().yaw;

    // 最終時刻の位置に対するゴールの方位
    // +の向きをroombaに合わせる
    float goal_theta = -std::atan2(local_goal_.pose.position.y - traj.back().y, local_goal_.pose.position.x - traj.back().x);

    // ゴールまでの方位差分
    float target_theta = 0.0;
    if(goal_theta > theta)
        target_theta = goal_theta - theta;
    else
        target_theta = theta - goal_theta;

    // headingの評価値
    float heading_eval = (M_PI - std::abs(target_theta))/M_PI; // 正規化

    return heading_eval;
}

// distの評価関数を計算
float LocalPathPlanner::calc_dist_eval(std::vector<State> traj)
{
    float search_range = 10.0;       // 探索する範囲
    float min_dist = search_range; // 最も近い障害物との距離

    // 最小値の更新
    for(auto& state : traj)
    {
        for(auto& ob_pose : ob_poses_.poses)
        {
            float dx = ob_pose.position.x - state.x;
            float dy = ob_pose.position.y - state.y;
            float dist = sqrt(powf(dx, 2.0)+powf(dy, 2.0));

            if(dist < min_dist)
                min_dist = dist;
        }
    }

    return min_dist/serach_range; // 正規化
}

// velocityの評価関数を計算
float LocalPathPlanner::calc_vel_eval(std::vector<State> traj)
{
    return std::abs(traj.back().velocity)/max_vel_;
}

// ゴールに着くまでTrueを返す
bool can_move()
{

}

void LocalPathPlanner::process()
{
    ros::Rate loop_rate(hz_); // 制御周波数の設定

    while(ros::ok())
    {
        if(can_move())
        {
            std::vector<float> input = calc_final_input();
            roomba_control(input[0], input[1]);
        }
        else
        {
            break;
        }

        ros::spinOnce();
        loop_rate.sleep(); // 周期が終わるまで待つ
    }
}


// main文
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "local_path_planner"); // ノードの初期化
    LocalPathPlanner local_path_planner;
    local_path_planner.process();

    return 0;
}