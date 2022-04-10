#ifndef LOCALIZER_H
#define LOCALIZER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LasorScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2/utils.h>
#include <vector>
#include <random>

#include "roomba_500driver_meiji/RoombaCtrl.h"


class Particle
{
public:
    Particle(double x=0,double y=0,double yaw=0,double weight=0);　// コンストラクタ(メンバ変数のセット)

    // メンバ変数の値を更新する関数
    void set_pose(double x,double y,double yaw);
    void set_weight(double weight);

    // メンバ変数の値を返す関数
    double get_pose_x(){return x_;}
    double get_pose_y(){return y_;}
    double get_pose_yaw(){return yaw_;}
    double get_weight(){return weight_;}

private:
    double x_;
    double y_;
    double yaw_;
    double weight_;
};

class ParticleFilter
{
public:
    ParticleFilter();　                                                      // コンストラクタ(メンバ変数のセット)
    void initialize();                                                       // パーティクルの新規作成
    void motion_update(nav_msgs::Odometry last,nav_msgs::Odometry prev);     // パーティクルの移動量を算出
    void move(Particle* p,double distance,double direction,double rotation); // パーティクルを移動
    void measurement_update();　　　　　　　　　　　　　　　　　　　　　　　 // 観測更新(センサの値と比較して重みを更新)
    void resampling();                                                       // リサンプリング
    void check_N();　　　　　　　　　　　　　　　　　　　　　　　　　　　　　// ゼロ割対策

    std::vector<Particle> particles_;　　　　　　　　　　　　　　　　　　　　// N 個の Particle を格納するベクタ

private:
    double set_noise(double mu,double cov); // ノイズを乗せる関数
    double optimize_angle(double angle);　　// 適切な角度 (-π ~ π) を返す
    double get_max_weight();　　　　　　　　// particles_ が持つ最大の重みを取得
    void normalize_weight();　　　　　　　　// 重みの正規化
    void reset_weight();　　　　　　　　　　// 重みを 1/N にする(リサンプリング時の処理)

    double num_; // パーティクル数

    // pose 初期化用の値　　　　　　　　　　　　　　　　
    double init_x_;
    double init_y_;
    double init_yaw_;

    // ノイズを乗せる際の標準偏差
    double x_cov_;
    double y_cov_;
    double yaw_cov_;

    Localizer* pMcl_; // Localizer から初期値を得るためのポインタ
};

class Localizer
{
public:
    Localizer();　　// コンストラクタ(パラメータのセット)
    void process();

    // メンバ変数の値を返す関数
    double get_num(){return num_;}
    double get_init_x(){return init_x_;}
    double get_init_y(){return init_y_;}
    double get_init_yaw(){return init_yaw_;}
    double get_init_x_cov(){return init_x_cov_;}
    double get_init_y_cov(){return init_y_cov_;}
    double get_init_yaw_cov(){return init_yaw_cov_;}

private:
    //各種コールバック関数
    void odometry_callback(nav_msgs::Odometry::ConstPtr &msg);
    void map_callback(nav_msgs::OccupancyGrid::ConstPtr &msg);
    void laser_callback(sensor_msgs::LaserScan::ConstPtr &msg);

    int hz_; // ループ周波数

    // 各種初期値
    int init_num_;
    double init_x_;
    double init_y_;
    double init_yaw_;
    double init_x_cov_;
    double init_y_cov_;
    double init_yaw_cov_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber sub_laser_;
    ros::Subscriber sub_odometry_;
    ros::Subscriber sub_map_;

    ros::Publisher pub_estimated_pose_;
    ros::Publisher pub_particle_cloud_;

    geometry_msgs::PoseStamped estimated_pose_; // 一意に推定した pose
    geometry_msgs::PoseArray particle_cloud_;　 // パーティクルの pose

    nav_msgs::Odometry last_odometry_; // 最新のオドメトリ
    nav_msgs::Odometry prev_odometry_; // 前のオドメトリ
    nav_msgs::OccupancyGrid map_;

    sensor_msgs::LasorScan laser_;

};

#endif
