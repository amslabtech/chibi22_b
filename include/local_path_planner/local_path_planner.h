#ifndef LOCAL_PATH_PLANNER
#define LOCAL_PATH_PLANNER

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>

#include "roomba_500driver_meiji/RoombaCtrl.h"


struct RoombaState // Roombaの状態を保持
{
    float x; // [m]
    float y; // [m]
    float yaw; // [rad]
    float velocity; // [m/s]
    float yawrate; // [rad/s]
};

class LocalPathPlanner
{
public:
    LocalPathPlanner(); // デフォルトコンストラクタ
    void process();

private:
    void local_map_callback(const nav_msgs::OccupancyGrid::ConstPtr&);        // local_mapのコールバック関数
    void local_goal_callback(const geometry_msgs::PoseStamped::ConstPtr&); // local_goalのコールバック関数
    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr&);        // poseのコールバック関数
    void roomba_control(float velocity, float yawrate);

    float calc_evaluation();
    float calc_heading_eval();
    float calc_dist_eval();
    float calc_vel_eval();
    void move();
    void calc_dynamic_window();
    void calc_trajectory();
    void calc_final_input();
    void dwa_control();

    int hz_; // ループ周波数
    float max_vel_; // 最高並進速度 [m/s]
    float min_vel_; // 最低並進速度 [m/s]
    float max_yawrate_; // 最高旋回速度 [rad/s]
    float max_accel_; // 最高並進加速度 [m/s^2]
    float max_dyawrate_; // 最高旋回加速度 [rad/s^2]
    float vel_reso_; // 並進速度解像度 [m/s]
    float yawrate_reso_; // 旋回速度解像度 [rad/s]
    float dt_; // 微小時間 [s]
    float predict_time_; // [s]

    // 重み定数
    float weight_heading_;
    float weight_dist_;
    float weight_vel_;
    // float roomba_radius_;


    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber sub_local_map_;                        // サブスクライバ（local_map）
    ros::Subscriber sub_local_goal_;                        // サブスクライバ（local_goal）
    ros::Subscriber sub_pose_;                               // サブスクライバ（現在位置）
    ros::Publisher pub_cmd_vel_;                            // パブリッシャ（制御入力）

    nav_msgs::OccupancyGrid local_map_;             // local map
    geometry_msgs::PoseStamped local_goal_;         // local path用の目標位置
    geometry_msgs::PoseStamped current_pose_; // 現在位置
    geometry_msgs::PoseStamped previous_pose_;       // 微小時間前の位置

    roomba_500driver_meiji::RoombaCtrl cmd_vel_; // 制御入力
    RoombaState roomba_;
};

#endif