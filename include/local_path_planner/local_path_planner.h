/*
速度について以下のようにする
velocity(vel) : 並進速度
yawrate       : 旋回速度
speed         : 速度の総称(vel, yawrate)
*/

#ifndef LOCAL_PATH_PLANNER
#define LOCAL_PATH_PLANNER

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2/utils.h>

#include "roomba_500driver_meiji/RoombaCtrl.h"


// ===== 構造体 =====
struct State
{
    float x;        // [m]
    float y;        // [m]
    float yaw;      // [rad]
    float velocity; // [m/s]
    float yawrate;  // [rad/s]
};

struct DynamicWindow
{
    float min_vel;
    float max_vel;
    float min_yawrate;
    float max_yawrate;
};


// ===== クラス =====
class Robot
{
public:
    Robot(); // デフォルトコンストラクタ
    void update_pose(const geometry_msgs::PoseStamped pose);   // poseの更新
    void set_speed(const float velocity, const float yawrate); // 直前の制御入力を記録

    // メンバ関数の値を返却する関数
    float x();
    float y();
    float yaw();
    float velocity();
    float yawrate();

private:
    State state_;
};

class LocalPathPlanner
{
public:
    LocalPathPlanner(); // デフォルトコンストラクタ
    void process();

private:
    // ----- 引数あり関数 -----
    // 各種コールバック関数
    void local_goal_callback(const geometry_msgs::PointStamped::ConstPtr&);
    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr&);
    void ob_poses_callback(const geometry_msgs::PoseArray::ConstPtr&);
    // void local_map_callback(const nav_msgs::OccupancyGrid::ConstPtr&);

    void  roomba_control(const float velocity, const float yawrate);     // Roombaの制御入力
    void  move(State& state, const float velocity, const float yawrate); // 予測軌跡作成時における仮想ロボットの移動
    void  calc_trajectory(const float velocity, const float yawrate);    // 予測軌跡の作成
    float calc_evaluation(const std::vector<State> traj);                // 評価関数を計算
    float calc_heading_eval(const std::vector<State> traj);              // headingの評価関数を計算
    float calc_dist_eval(const std::vector<State> traj);                 // distの評価関数を計算
    float calc_vel_eval(const std::vector<State> traj);                  // velocityの評価関数を計算


    // ----- 引数なし関数 -----
    void calc_dynamic_window(); // Dynamic Windowを計算
    void calc_final_input();    // 最適な制御入力を計算
    bool can_move();            // ゴールに着くまでTrueを返す
    // void dwa_control();


    // ----- 変数 -----
    int   hz_;             // ループ周波数
    float max_vel_;        // 最高並進速度 [m/s]
    float min_vel_;        // 最低並進速度 [m/s]
    float max_yawrate_;    // 最高旋回速度 [rad/s]
    float max_accel_;      // 最高並進加速度 [m/s^2]
    float max_dyawrate_;   // 最高旋回加速度 [rad/s^2]
    float vel_reso_;       // 並進速度解像度 [m/s]
    float yawrate_reso_;   // 旋回速度解像度 [rad/s]
    float dt_;             // 微小時間 [s]
    float predict_time_;   // 軌跡予測時間 [s]
    float roomba_radius_;  // Roombaのサイズ [m]（Roombaの中心から壁までの最小距離）
    float goal_tolerance_; // 目標地点の許容誤差 [m]

    // 重み定数
    float weight_heading_;
    float weight_dist_;
    float weight_vel_;


    // ----- オブジェクト -----
    Robot roomba_;
    DynamicWindow dw_;

    // NodeHandle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // Subscriber
    ros::Subscriber sub_local_goal_;
    ros::Subscriber sub_pose_;
    ros::Subscriber sub_ob_poses_;
    // ros::Subscriber sub_local_map_;

    // Publisher
    ros::Publisher pub_cmd_speed_;

    // pose関連
    geometry_msgs::PointStamped local_goal_;   // local path用の目標位置
    geometry_msgs::PoseStamped  current_pose_; // 現在位置
    geometry_msgs::PoseArray    ob_poses_;     // 障害物のポーズの配列
    // nav_msgs::OccupancyGrid local_map_;
    // geometry_msgs::PoseStamped previous_pose_; // 微小時間前の位置

    // 制御入力
    roomba_500driver_meiji::RoombaCtrl cmd_speed_;
};

#endif
