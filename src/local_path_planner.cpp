/*
速度について以下のようにする
velocity(vel) : 並進速度
yawrate       : 旋回速度
speed         : 速度の総称(vel, yawrate)
*/

#include "local_path_planner/local_path_planner.h"

// コンストラクタ
DWA::DWA():private_nh_("~")
{
    // パラメータの取得
    private_nh_.getParam("is_visible", is_visible_);
    private_nh_.getParam("hz", hz_);
    private_nh_.getParam("max_vel1", max_vel1_);
    private_nh_.getParam("max_vel2", max_vel2_);
    private_nh_.getParam("avoid_thres_vel", avoid_thres_vel_);
    private_nh_.getParam("min_vel", min_vel_);
    private_nh_.getParam("max_yawrate1", max_yawrate1_);
    private_nh_.getParam("max_yawrate2", max_yawrate2_);
    private_nh_.getParam("turn_thres_yawrate", turn_thres_yawrate_);
    private_nh_.getParam("mode_log_time", mode_log_time_);
    private_nh_.getParam("max_accel", max_accel_);
    private_nh_.getParam("max_dyawrate", max_dyawrate_);
    private_nh_.getParam("vel_reso", vel_reso_);
    private_nh_.getParam("yawrate_reso", yawrate_reso_);
    private_nh_.getParam("dt", dt_);
    private_nh_.getParam("predict_time1", predict_time1_);
    private_nh_.getParam("predict_time2", predict_time2_);
    private_nh_.getParam("roomba_radius", roomba_radius_);
    private_nh_.getParam("radius_margin1", radius_margin1_);
    private_nh_.getParam("radius_margin2", radius_margin2_);
    private_nh_.getParam("goal_tolerance", goal_tolerance_);
    private_nh_.getParam("search_range", search_range_);
    private_nh_.getParam("weight_heading1", weight_heading1_);
    private_nh_.getParam("weight_heading2", weight_heading2_);
    private_nh_.getParam("weight_dist1", weight_dist1_);
    private_nh_.getParam("weight_dist2", weight_dist2_);
    private_nh_.getParam("weight_vel", weight_vel_);

    // Subscriber
    sub_local_goal_ = nh_.subscribe("/local_goal", 1, &DWA::local_goal_callback, this);
    sub_ob_poses_   = nh_.subscribe("/local_map/obstacle", 1, &DWA::ob_poses_callback, this);

    // Publisher
    pub_cmd_speed_    = nh_.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control", 1);
    pub_predict_path_ = nh_.advertise<nav_msgs::Path>("/predict_local_paths", 1);
    pub_optimal_path_ = nh_.advertise<nav_msgs::Path>("/optimal_local_path", 1);
}

// local_goalのコールバック関数
void DWA::local_goal_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    geometry_msgs::TransformStamped transform;
    try
    {
        transform = tf_buffer_.lookupTransform("base_link", "map", ros::Time(0));
        flag_local_goal_ = true;
    }
    catch(tf2::TransformException& ex)
    {
        ROS_WARN("%s", ex.what());
        flag_local_goal_ = false;
        return;
    }
    tf2::doTransform(*msg, local_goal_, transform);
}

// ob_posesのコールバック関数
void DWA::ob_poses_callback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    ob_poses_ = *msg;
    flag_ob_poses_ = true;
}

// 唯一main文で実行する関数
void DWA::process()
{
    ros::Rate loop_rate(hz_); // 制御周波数の設定
    tf2_ros::TransformListener tf_listener(tf_buffer_);

    while(ros::ok())
    {
        if(can_move())
        {
            const std::vector<double> input = calc_final_input();
            roomba_control(input[0], input[1]);
        }
        else
        {
            roomba_control(0.0, 0.0);
        }

        ros::spinOnce();   // コールバック関数の実行
        loop_rate.sleep(); // 周期が終わるまで待つ
    }
}

// ゴールに着くまでTrueを返す
bool DWA::can_move()
{
    if(!(flag_local_goal_ && flag_ob_poses_)) return false; // msg受信済みか

    const double dx = local_goal_.point.x;
    const double dy = local_goal_.point.y;
    const double dist_to_goal = hypot(dx, dy); // 現在位置からゴールまでの距離

    if(dist_to_goal > goal_tolerance_)
        return true;
    else
        return false;
}

// Roombaの制御入力を行う
void DWA::roomba_control(const double velocity, const double yawrate)
{
    cmd_speed_.mode           = 11; // 任意の動作を実行するモード
    cmd_speed_.cntl.linear.x  = velocity;
    cmd_speed_.cntl.angular.z = yawrate;

    pub_cmd_speed_.publish(cmd_speed_);
}

// 最適な制御入力を計算
std::vector<double> DWA::calc_final_input()
{
    std::vector<double> input{0.0, 0.0};          // {velocity, yawrate}
    std::vector<std::vector<State>> trajectories; // すべての軌跡格納用
    double max_score = -1e6;                      // 評価値の最大値格納用
    int index_of_max_score = 0;                   // 評価値の最大値に対する軌跡のインデックス格納用

    // 旋回状況に応じた減速機能
    change_mode();

    // ダイナミックウィンドウを計算
    calc_dynamic_window();

    // 並進速度と旋回速度のすべての組み合わせを評価
    int i = 0; // 現在の軌跡のインデックス保持用
    for(double velocity=dw_.min_vel; velocity<=dw_.max_vel; velocity+=vel_reso_)
    {
        for(double yawrate=dw_.min_yawrate; yawrate<=dw_.max_yawrate; yawrate+=yawrate_reso_)
        {

            if(velocity<vel_reso_*0.5 && abs(yawrate)<yawrate_reso_*4.5)
                continue;
            else if(yawrate_reso_*0.5 < yawrate && yawrate<=yawrate_reso_*1.5 && mode_==1)
                continue;
            else if(-yawrate_reso_*5.5 < yawrate && yawrate<=-yawrate_reso_*0.5 && mode_==1)
                continue;

            const std::vector<State> trajectory = calc_traj(velocity, yawrate); // 予測軌跡の生成
            const double score = calc_evaluation(trajectory); // 予測軌跡に対する評価値の計算
            trajectories.push_back(trajectory);

            // 最大値の更新
            if(max_score < score)
            {
                max_score = score;
                input[0]  = velocity;
                input[1]  = yawrate;
                index_of_max_score = i;
            }
            i++;
        }
    }

    // 現在速度の記録
    roomba_.velocity = input[0];
    roomba_.yawrate  = input[1];

    // pathの可視化
    if(is_visible_)
    {
        ros::Time now = ros::Time::now();
        for(i=0; i<trajectories.size(); i++)
        {
            if(i == index_of_max_score)
                visualize_traj(trajectories[i], pub_optimal_path_, now);
            else
                visualize_traj(trajectories[i], pub_predict_path_, now);
        }
    }

    return input;
}

// 旋回状況に応じた減速機能
void DWA::change_mode()
{
    if(abs(roomba_.yawrate)>turn_thres_yawrate_ || roomba_.velocity<avoid_thres_vel_)
        mode_log_.push_back(2.0); // 減速モード
    else
        mode_log_.push_back(1.0);

    if(mode_log_.size() > hz_*mode_log_time_)
        mode_log_.erase(mode_log_.begin());

    double mode_sum = 0.0;
    for(const auto& mode : mode_log_)
    {
        mode_sum += mode;
    }

    double mode_avg = mode_sum/mode_log_.size();

    if(mode_avg < 1.5) // 平常時
    {
        mode_          = 1;
        max_vel_        = max_vel1_;
        max_yawrate_    = max_yawrate1_;
        radius_margin_  = radius_margin1_;
        weight_heading_ = weight_heading1_;
        weight_dist_    = weight_dist1_;
        predict_time_   = predict_time1_;
        // std::cout << "減速OFF" << std::endl;
    }
    else // 減速時
    {
        mode_          = 2;
        max_vel_        = max_vel2_;
        max_yawrate_    = max_yawrate2_;
        radius_margin_  = radius_margin2_;
        weight_heading_ = weight_heading2_;
        weight_dist_    = weight_dist2_;
        predict_time_   = predict_time2_;
        // std::cout << "減速ON" << std::endl;
    }
}

// Dynamic Windowを計算
void DWA::calc_dynamic_window()
{
    // 車両モデルによるWindow
    double Vs[] = {min_vel_, max_vel_, -max_yawrate_, max_yawrate_};

    // 運動モデルによるWindow
    double Vd[] = {roomba_.velocity - max_accel_*dt_,
                   roomba_.velocity + max_accel_*dt_,
                   roomba_.yawrate  - max_dyawrate_*dt_,
                   roomba_.yawrate  + max_dyawrate_*dt_};

    // 最終的なDynamic Window
    dw_.min_vel     = std::max(Vs[0], Vd[0]);
    dw_.max_vel     = std::min(Vs[1], Vd[1]);
    dw_.min_yawrate = std::max(Vs[2], Vd[2]);
    dw_.max_yawrate = std::min(Vs[3], Vd[3]);
}

// 予測軌跡を作成
std::vector<State> DWA::calc_traj(const double velocity, const double yawrate)
{
    std::vector<State> trajectory;           // 軌跡格納用の動的配列
    State state = {0.0, 0.0, 0.0, 0.0, 0.0}; // 軌跡作成用の仮想ロボット

    // 軌跡を格納
    for(double t=0.0; t<=predict_time_; t+=dt_)
    {
        move(state, velocity, yawrate);
        trajectory.push_back(state);
    }

    return trajectory;
}

// 予測軌跡作成時における仮想ロボットを移動
void DWA::move(State& state, const double velocity, const double yawrate)
{
    state.yaw      += yawrate * dt_;
    state.yaw       = optimize_angle(state.yaw);
    state.x        += velocity * cos(state.yaw) * dt_;
    state.y        += velocity * sin(state.yaw) * dt_;
    state.velocity  = velocity;
    state.yawrate   = yawrate;
}

// 適切な角度(-M_PI ~ M_PI)を返す
double DWA::optimize_angle(double angle)
{
    if(M_PI  < angle) angle -= 2.0*M_PI;
    if(angle < -M_PI) angle += 2.0*M_PI;

    return angle;
}

// 評価関数を計算
double DWA::calc_evaluation(const std::vector<State>& traj)
{
    const double heading_score  = weight_heading_ * calc_heading_eval(traj);
    const double distance_score = weight_dist_    * calc_dist_eval(traj);
    const double velocity_score = weight_vel_     * calc_vel_eval(traj);

    const double total_score = heading_score + distance_score + velocity_score;

    return total_score;
}

// headingの評価関数を計算
double DWA::calc_heading_eval(const std::vector<State>& traj)
{
    // 最終時刻のロボットの方位
    const double theta = traj.back().yaw;

    // 最終時刻の位置に対するゴールの方位
    const double goal_theta = atan2(local_goal_.point.y - traj.back().y, local_goal_.point.x - traj.back().x);

    // ゴールまでの方位差分
    double target_theta = 0.0;
    if(goal_theta > theta)
        target_theta = goal_theta - theta;
    else
        target_theta = theta - goal_theta;

    // headingの評価値
    const double heading_eval = (M_PI - abs(optimize_angle(target_theta)))/M_PI; // 正規化

    return heading_eval;
}

// distの評価関数を計算
double DWA::calc_dist_eval(const std::vector<State>& traj)
{
    double min_dist = search_range_; // 最も近い障害物との距離

    // pathの点と障害物のすべての組み合わせを探索
    for(const auto& state : traj)
    {
        for(const auto& ob_pose : ob_poses_.poses)
        {
            // pathのうちの１点と障害物の距離を計算
            const double dx   = ob_pose.position.x - state.x;
            const double dy   = ob_pose.position.y - state.y;
            const double dist = hypot(dx, dy);

            // 壁に衝突したパスを評価
            if(dist <= roomba_radius_+radius_margin_)
                return -1e6;

            // 最小値の更新
            if(dist < min_dist)
                min_dist = dist;
        }
    }

    return min_dist/search_range_; // 正規化
}

// velocityの評価関数を計算
double DWA::calc_vel_eval(const std::vector<State>& traj)
{
    if(0.0 < traj.back().velocity) // 前進
        return traj.back().velocity/max_vel_; // 正規化
    else // 後退
        return 0.0;
}

// 軌跡を可視化
void DWA::visualize_traj(const std::vector<State>& traj, const ros::Publisher& pub_local_path, ros::Time now)
{
    nav_msgs::Path local_path;
    local_path.header.stamp = now;
    local_path.header.frame_id = "base_link";

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = now;
    pose.header.frame_id = "base_link";

    for(const auto& state : traj)
    {
        pose.pose.position.x = state.x;
        pose.pose.position.y = state.y;
        local_path.poses.push_back(pose);
    }

    pub_local_path.publish(local_path);
}
