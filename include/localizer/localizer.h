#ifndef LOCALIZER_H
#define LOCALIZER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2/utils.h>
#include <tf/transform_broadcaster.h>
#include <vector>
#include <algorithm>
#include <random>

#include "roomba_500driver_meiji/RoombaCtrl.h"


class Particle
{
public:
    Particle(double x=0.0,double y=0.0,double yaw=0.0,double weight=0.0); // コンストラクタ(メンバ変数のセット)

    // メンバ変数の値を更新する関数
    void set_pose(double x,double y,double yaw);
    void set_weight(double weight);

    // メンバ変数の値を返す関数
    double get_pose_x() const {return x_;}
    double get_pose_y() const {return y_;}
    double get_pose_yaw() const {return yaw_;}
    double get_weight() const {return weight_;}

    bool operator<(const Particle& another) const {return weight_ < another.weight_;}

private:
    double x_;
    double y_;
    double yaw_;
    double weight_;
};


class Localizer
{
public:
    Localizer(); // コンストラクタ(パラメータのセット)
    void process();

private:
    //各種コールバック関数
    void odometry_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg);

    void initialize(); // パーティクルの新規作成
    void motion_update(nav_msgs::Odometry last,nav_msgs::Odometry prev); // パーティクルの移動量を算出
    void move(Particle& p,double distance,double direction,double rotation); // パーティクルを移動
    void measurement_update(); // 観測更新(センサの値と比較して重みを更新)
    bool reset_request();
    void resampling(); // リサンプリング
    void expansion_reset();
    void estimate_pose();

    void publish_particles();
    void publish_estimated_pose();
    void broadcast_roomba_state();

    double set_noise(double mu,double cov); // ノイズを乗せる関数
    double optimize_angle(double angle);    // 適切な角度 (-π ~ π) を返す

    double calc_weight(Particle& p);
    double likelihood(double x,double mu,double sigma);
    double dist_on_map(double map_x,double map_y,const double laser_dist,const double lasor_angle);
    int get_map_occupancy(double x,double y);


    double get_max_weight();                // particles_ が持つ最大の重みを取得
    void normalize_weight();                // 重みの正規化
    void reset_weight();                    // 重みを 1/N にする(リサンプリング時の処理)

    int hz_; // ループ周波数

    int num_ = 0;
    double init_x_ = 0.0;
    double init_y_ = 0.0;
    double init_yaw_ = 0.0;
    double x_cov_ = 0.0;
    double y_cov_ = 0.0;
    double yaw_cov_ = 0.0;
    double distance_noise_rate_ = 0.0;
    double rotation_noise_rate_ = 0.0;
    double laser_noise_rate_ = 0.0;
    int laser_step_ = 0.0;
    double laser_ignore_range_ = 0.0;
    double alpha_slow_th_ = 0.0;
    double alpha_fast_th_ = 0.0;
    double expansion_rate_th_ = 0.0;

    double alpha_ = 0.0;
    double alpha_slow_ = 0.0;
    double alpha_fast_ = 0.0;
    int num_replace_ = 0;


    bool init_request_ = true;  // 初期化実行用フラグ
    bool odometry_got_ = false; // odometry を取得したか
    bool map_got_ = false;
    bool laser_got_ = false;

    Particle estimated_pose_;
    std::vector<Particle> particles_; // N 個の Particle を格納するベクタ

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber sub_laser_;
    ros::Subscriber sub_odometry_;
    ros::Subscriber sub_map_;

    ros::Publisher pub_estimated_pose_;
    ros::Publisher pub_particle_cloud_;

    tf::TransformBroadcaster roomba_state_broadcaster_;

    geometry_msgs::PoseStamped estimated_pose_msg_; // 一意に推定した pose
    geometry_msgs::PoseArray particle_cloud_msg_;   // パーティクルの pose

    nav_msgs::Odometry last_odometry_; // 最新のオドメトリ
    nav_msgs::Odometry prev_odometry_; // 前のオドメトリ
    nav_msgs::OccupancyGrid map_;

    sensor_msgs::LaserScan laser_;

};

#endif
