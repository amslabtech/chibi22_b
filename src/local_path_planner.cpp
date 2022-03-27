/*
速度について以下のようにする
velocity(vel) : 並進速度
yawrate       : 旋回速度
speed         : 速度の総称(vel, yawrate)
*/

#include "local_path_planner/local_path_planner.h"

// ===== クラスRobot =====
// コンストラクタ
Robot::Robot()
{
    state_.x        = 0.0;
    state_.y        = 0.0;
    state_.yaw      = 0.0;
    state_.velocity = 0.0;
    state_.yawrate  = 0.0;
}

// poseを更新
void Robot::update_pose(const geometry_msgs::PoseStamped pose)
{
    state_.x   = pose.pose.position.x;
    state_.y   = pose.pose.position.y;
    state_.yaw = tf2::getYaw(pose.pose.orientation);
}

// 直前の制御入力を記録
void Robot::set_speed(const float velocity, const float yawrate)
{
    state_.velocity = velocity;
    state_.yawrate  = yawrate;
}

// メンバ関数の値を返却する関数
float Robot::x()        { return state_.x; }
float Robot::y()        { return state_.y; }
float Robot::yaw()      { return state_.yaw; }
float Robot::velocity() { return state_.velocity; }
float Robot::yawrate()  { return state_.yawrate; }


// ===== クラスLocalPathPlanner =====
// コンストラクタ
LocalPathPlanner::LocalPathPlanner():private_nh_("~"), nh_("")
{
    // パラメータの取得
    private_nh_.getParam("hz", hz_);
    private_nh_.getParam("max_vel", max_vel_);
    private_nh_.getParam("min_vel", min_vel_);
    private_nh_.getParam("max_yawrate", max_yawrate_);
    private_nh_.getParam("max_accel", max_accel_);
    private_nh_.getParam("max_dyawrate", max_dyawrate_);
    private_nh_.getParam("vel_reso", vel_reso_);
    private_nh_.getParam("yawrate_reso", yawrate_reso_);
    private_nh_.getParam("dt", dt_);
    private_nh_.getParam("predict_time", predict_time_);
    private_nh_.getParam("weight_heading", weight_heading_);
    private_nh_.getParam("weight_dist", weight_dist_);
    private_nh_.getParam("weight_vel", weight_vel_);
    private_nh_.getParam("goal_tolerance", goal_tolerance_);

    // Subscriber
    sub_local_goal_ = nh_.subscribe("/local_goal", 10, &LocalPathPlanner::local_goal_callback, this);
    sub_pose_       = nh_.subscribe("/roomba/pose", 10, &LocalPathPlanner::pose_callback, this);
    sub_ob_poses_   = nh_.subscribe("/local_map/obstacle", 10, &LocalPathPlanner::ob_poses_callback, this);
    // sub_local_map_  = nh_.subscribe("/local_map/local_map", 10, &LocalPathPlanner::local_map_callback, this);

    // Publisher
    pub_cmd_speed_ = nh_.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control", 1);
}

// local_goalのコールバック関数
void LocalPathPlanner::local_goal_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    local_goal_ = *msg;
}

// poseのコールバック関数
void LocalPathPlanner::pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose_ = *msg;
    roomba_.update_pose(current_pose_);
}

// ob_posesのコールバック関数
void LocalPathPlanner::ob_poses_callback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    ob_poses_ = *msg;
}

// local_mapのコールバック関数
// void LocalPathPlanner::local_map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
// {
//     local_map_ = *msg;
// }

// Roombaの制御入力を行う
void LocalPathPlanner::roomba_control(const float velocity, const float yawrate)
{
    cmd_speed_.mode           = 11; // 任意の動きを実行するモード
    cmd_speed_.cntl.linear.x  = velocity;
    cmd_speed_.cntl.angular.z = yawrate;

    pub_cmd_speed_.publish(cmd_speed_);
}

// 予測軌跡作成時における仮想ロボットを移動
void LocalPathPlanner::move(State& state, const float velocity, const float yawrate)
{
    state.yaw      += yawrate * dt_;
    state.x        += velocity * cos(state.yaw) * dt_;
    state.y        += velocity * sin(state.yaw) * dt_;
    state.velocity  = velocity;
    state.yawrate   = yawrate;
}

// Dynamic Windowを計算
void LocalPathPlanner::calc_dynamic_window()
{
    // 車両モデルによるWindow
    float Vs[] = {min_vel_, max_vel_, -max_yawrate_, max_yawrate_};

    // 運動モデルによるWindow
    float Vd[] = {roomba_.velocity() - max_accel_*dt_,
                  roomba_.velocity() + max_accel_*dt_,
                  roomba_.yawrate()  - max_dyawrate_*dt_,
                  roomba_.yawrate()  - max_dyawrate_*dt_};

    // 最終的なDynamic Window
    dw_.min_vel     = std::max(Vs[0], Vd[0]);
    dw_.max_vel     = std::min(Vs[1], Vd[1]);
    dw_.min_yawrate = std::max(Vs[2], Vd[2]);
    dw_.max_yawrate = std::min(Vs[3], Vd[3]);
}

// 予測軌跡を作成
std::vector<State> LocalPathPlanner::calc_trajectory(const float velocity, const float yawrate)
{
    std::vector<State> trajectory;           // 軌跡格納用の動的配列
    State state = {0.0, 0.0, 0.0, 0.0, 0.0}; // 軌跡作成用の仮想ロボット

    // 現在のposeを代入
    // state.x   = roomba_.x();
    // state.y   = roomba_.y();
    // state.yaw = roomba_.yaw();

    // 軌跡を格納
    for(double t=0.0; t<=predict_time_; t+=dt_)
    {
        move(state, velocity, yawrate);
        trajectory.push_back(state);
    }

    return trajectory;
}

// 最適な制御入力を計算
std::vector<float> LocalPathPlanner::calc_final_input()
{
    std::vector<float> input{0.0, 0.0}; // {velocity, yawrate}
    float max_score = 0.0;              // 評価値の最大値格納用
    calc_dynamic_window();              // ダイナミックウィンドウを計算

    // 並進速度と旋回速度のすべての組み合わせを評価
    for(float velocity=dw_.min_vel; velocity<=dw_.max_vel; velocity+=vel_reso_)
    {
        for(float yawrate=dw_.min_yawrate; yawrate<=dw_.max_yawrate; yawrate+=yawrate_reso_)
        {
            const std::vector<State> trajectory = calc_trajectory(velocity, yawrate); // 予測軌跡の生成
            const float score = calc_evaluation(trajectory); // 予測軌跡に対する評価値の計算

            // 最大値の更新
            if(max_score < score)
            {
                max_score = score;
                input[0]  = velocity;
                input[1]  = yawrate;
            }
        }
    }

    roomba_.set_speed(input[0], input[1]);

    return input;
}

// 評価関数を計算
float LocalPathPlanner::calc_evaluation(const std::vector<State> traj)
{
    const float heading_score  = weight_heading_ * calc_heading_eval(traj);
    const float distance_score = weight_dist_    * calc_dist_eval(traj);
    const float velocity_score = weight_vel_     * calc_vel_eval(traj);

    const float total_score    = heading_score + distance_score + velocity_score;

    return total_score;
}

// headingの評価関数を計算
float LocalPathPlanner::calc_heading_eval(const std::vector<State> traj)
{
    // 最終時刻のロボットの方位
    const float theta = traj.back().yaw;

    // 最終時刻の位置に対するゴールの方位(+の向きをroombaに合わせる)
    const float goal_theta = -atan2(local_goal_.point.y - traj.back().y, local_goal_.point.x - traj.back().x);

    // ゴールまでの方位差分
    float target_theta = 0.0;
    if(goal_theta > theta)
        target_theta = goal_theta - theta;
    else
        target_theta = theta - goal_theta;

    // headingの評価値
    const float heading_eval = (M_PI - abs(target_theta))/M_PI; // 正規化

    return heading_eval;
}

// distの評価関数を計算
float LocalPathPlanner::calc_dist_eval(const std::vector<State> traj)
{
    const float search_range = 10.0; // 探索する範囲
    float min_dist = search_range;   // 最も近い障害物との距離

    // pathの点と障害物のすべての組み合わせを探索
    for(auto& state : traj)
    {
        for(auto& ob_pose : ob_poses_.poses)
        {
            const float dx   = ob_pose.position.x - state.x;
            const float dy   = ob_pose.position.y - state.y;
            const float dist = sqrt(powf(dx, 2.0)+powf(dy, 2.0)); // pathのうちの１点と障害物の距離

            // 最小値の更新
            if(dist < min_dist)
                min_dist = dist;
        }
    }

    return min_dist/search_range; // 正規化
}

// velocityの評価関数を計算
float LocalPathPlanner::calc_vel_eval(const std::vector<State> traj)
{
    return abs(traj.back().velocity)/max_vel_; // 正規化
}

// ゴールに着くまでTrueを返す
bool LocalPathPlanner::can_move()
{
    const float dx = local_goal_.point.x - roomba_.x();
    const float dy = local_goal_.point.y - roomba_.y();
    const float dist_to_goal = sqrt(powf(dx, 2.0) + powf(dy, 2.0)); // 現在位置からゴールまでの距離

    if(dist_to_goal > goal_tolerance_)
        return true;
    else
        return false;
}

void LocalPathPlanner::process()
{
    ros::Rate loop_rate(hz_); // 制御周波数の設定

    while(ros::ok())
    {
        if(can_move())
        {
            const std::vector<float> input = calc_final_input();
            roomba_control(input[0], input[1]);
        }
        else
        {
            roomba_control(0.0, 0.0);
            break;
        }

        ros::spinOnce();   // コールバック関数の実行
        loop_rate.sleep(); // 周期が終わるまで待つ
    }
}


//===== メイン関数 =====
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "local_path_planner"); // ノードの初期化
    LocalPathPlanner local_path_planner;
    local_path_planner.process();

    return 0;
}
