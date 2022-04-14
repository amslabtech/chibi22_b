#ifndef LOCALIZER_H
#define LOCALIZER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2/utils.h>
#include <tf2/convert.h>
#include <vector>
#include <random>

struct Parameter
{
    int num_ = 0;
    double init_x_ = 0.0;
    double init_y_ = 0.0;
    double init_yaw_ = 0.0;
    double x_cov_ = 0.0;
    double y_cov_ = 0.0;
    double yaw_cov_ = 0.0;
    double distance_noise_ratio_ = 0.0;
    double rotation_noise_ratio_ = 0.0;
};

class Particle
{
public:
    Particle(double x=0,double y=0,double yaw=0,double weight=0); // コンストラクタ(メンバ変数のセット)

    // メンバ変数の値を更新する関数
    void set_pose(double x,double y,double yaw);
    void set_weight(double weight);

    // メンバ変数の値を返す関数
    double get_pose_x() const {return x_;}
    double get_pose_y() const {return y_;}
    double get_pose_yaw() const {return yaw_;}
    double get_weight() const {return weight_;}

private:
    double x_;
    double y_;
    double yaw_;
    double weight_;
};

class ParticleFilter
{
public:
    ParticleFilter(Parameter param); // コンストラクタ(メンバ変数のセット)
    void set_parameter(Parameter param);
    void initialize(); // パーティクルの新規作成
    void motion_update(nav_msgs::Odometry last,nav_msgs::Odometry prev); // パーティクルの移動量を算出
    void move(Particle& p,double distance,double direction,double rotation); // パーティクルを移動
    void measurement_update(); // 観測更新(センサの値と比較して重みを更新)
    void resampling(); // リサンプリング
    void check_num(); // ゼロ割対策

    std::vector<Particle> particles_; // N 個の Particle を格納するベクタ

private:
    double set_noise(double mu,double cov); // ノイズを乗せる関数
    double optimize_angle(double angle);    // 適切な角度 (-π ~ π) を返す
    double get_max_weight();                // particles_ が持つ最大の重みを取得
    void normalize_weight();                // 重みの正規化
    void reset_weight();                    // 重みを 1/N にする(リサンプリング時の処理)

    Parameter param_;
};

class Localizer
{
public:
    Localizer(); // コンストラクタ(パラメータのセット)
    ~Localizer();
    void process();

private:
    //各種コールバック関数
    void odometry_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg);

    void publish_particles();

    int hz_; // ループ周波数

    bool init_request_ = true;  // 初期化実行用フラグ
    bool get_odometry_ = false; // odometry を取得したか

    Parameter init_param_;
    ParticleFilter* pf_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber sub_laser_;
    ros::Subscriber sub_odometry_;
    ros::Subscriber sub_map_;

    ros::Publisher pub_estimated_pose_;
    ros::Publisher pub_particle_cloud_;

    geometry_msgs::PoseStamped estimated_pose_msg_; // 一意に推定した pose
    geometry_msgs::PoseArray particle_cloud_msg_;   // パーティクルの pose

    nav_msgs::Odometry last_odometry_; // 最新のオドメトリ
    nav_msgs::Odometry prev_odometry_; // 前のオドメトリ
    nav_msgs::OccupancyGrid map_;

    sensor_msgs::LaserScan laser_;

};

#endif
