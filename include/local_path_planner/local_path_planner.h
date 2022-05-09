/*
速度について以下のようにする
velocity(vel) : 並進速度
yawrate       : 旋回速度
speed         : 速度の総称(vel, yawrate)
*/

#ifndef LOCAL_PATH_PLANNER_H
#define LOCAL_PATH_PLANNER_H

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

#include "roomba_500driver_meiji/RoombaCtrl.h"


// ===== 構造体 =====
struct State
{
    double x;        // [m]
    double y;        // [m]
    double yaw;      // [rad]
    double velocity; // [m/s]
    double yawrate;  // [rad/s]
};

struct DynamicWindow
{
    double min_vel;     // [m/s]
    double max_vel;     // [m/s]
    double min_yawrate; // [rad/s]
    double max_yawrate; // [rad/s]
};


// ===== クラス =====
class DWA
{
public:
    DWA(); // デフォルトコンストラクタ
    void process();

private:
    // ----- 引数あり関数 -----
    // 各種コールバック関数
    void local_goal_callback(const geometry_msgs::PointStamped::ConstPtr& msg);
    void ob_poses_callback(const geometry_msgs::PoseArray::ConstPtr& msg);

    // その他の関数
    void   roomba_control(const double velocity, const double yawrate);                                         // Roombaの制御入力
    void   move(State& state, const double velocity, const double yawrate);                                     // 予測軌跡作成時における仮想ロボットの移動
    void   visualize_traj(const std::vector<State>& traj, const ros::Publisher& pub_local_path, ros::Time now); // 軌跡を可視化
    double optimize_angle(double angle);                                                                        // 適切な角度(-M_PI ~ M_PI)を返す
    double calc_evaluation(const std::vector<State>& traj);                                                     // 評価関数を計算
    double calc_heading_eval(const std::vector<State>& traj);                                                   // headingの評価関数を計算
    double calc_dist_eval(const std::vector<State>& traj);                                                      // distの評価関数を計算
    double calc_vel_eval(const std::vector<State>& traj);                                                       // velocityの評価関数を計算
    std::vector<State> calc_traj(const double velocity, const double yawrate);                                  // 予測軌跡の作成


    // ----- 引数なし関数 -----
    void calc_dynamic_window();             // Dynamic Windowを計算
    void change_mode();
    bool can_move();                        // ゴールに着くまでTrueを返す
    std::vector<double> calc_final_input(); // 最適な制御入力を計算


    // ----- 変数 -----
    bool   is_visible_;         // パスを可視化するかの設定用
    int    hz_;                 // ループ周波数
    int    mode_;
    double max_vel_;            // 最高並進速度 [m/s]（計算用）
    double max_vel1_;           // 最高並進速度 [m/s]（平常時）
    double max_vel2_;           // 最高並進速度 [m/s]（減速時）
    double turn_thres_yawrate_; // 旋回中か判断する閾値
    double avoid_thres_vel_;    // 回避中か判断する閾値
    double mode_log_time_;      // logをとる時間区間
    double min_vel_;            // 最低並進速度 [m/s]
    double max_yawrate_;        // 最高旋回速度 [rad/s]（計算用）
    double max_yawrate1_;       // 最高旋回速度 [rad/s]（平常時）
    double max_yawrate2_;       // 最高旋回速度 [rad/s]（減速時）
    double max_accel_;          // 最高並進加速度 [m/s^2]
    double max_dyawrate_;       // 最高旋回加速度 [rad/s^2]
    double vel_reso_;           // 並進速度解像度 [m/s]
    double yawrate_reso_;       // 旋回速度解像度 [rad/s]
    double dt_;                 // 微小時間 [s]
    double predict_time_;       // 軌跡予測時間 [s]
    double predict_time1_;       // 軌跡予測時間 [s]
    double predict_time2_;       // 軌跡予測時間 [s]
    double roomba_radius_;      // Roombaのサイズ(半径) [m]
    double radius_margin_;      // 半径の余白 [m]（計算用）
    double radius_margin1_;     // 半径の余白 [m]（平常時）
    double radius_margin2_;     // 半径の余白 [m]（減速時）
    double goal_tolerance_;     // 目標地点の許容誤差 [m]
    double search_range_;       // 評価関数distで探索する範囲 [m]

    // msgの受け取り判定用
    bool flag_local_goal_ = false;
    bool flag_ob_poses_   = false;

    // mode記録用
    std::vector<double> mode_log_;

    // 重み定数
    double weight_heading_;  //（計算用）
    double weight_heading1_; //（平常時）
    double weight_heading2_; //（減速時）
    double weight_dist_;     //（計算用）
    double weight_dist1_;    //（平常時）
    double weight_dist2_;    //（減速時）
    double weight_vel_;


    // ----- その他のオブジェクト -----
    State roomba_;
    DynamicWindow dw_;

    // NodeHandle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // Subscriber
    ros::Subscriber sub_local_goal_;
    ros::Subscriber sub_ob_poses_;

    // Publisher
    ros::Publisher pub_cmd_speed_;
    ros::Publisher pub_optimal_path_;
    ros::Publisher pub_predict_path_;

    // pose関連
    geometry_msgs::PointStamped local_goal_; // local path用の目標位置
    geometry_msgs::PoseArray    ob_poses_;   // 障害物のポーズの配列

    // tf
    tf2_ros::Buffer tf_buffer_;

    // 制御入力
    roomba_500driver_meiji::RoombaCtrl cmd_speed_;
};

#endif
