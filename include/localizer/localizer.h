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

    double set_noise(double mu,double dev);
    double optimize_angle(double angle);

    void initialize();

    void motion_update();
    void move(Particle& p,double distance,double direction,double rotation);

    void measurement_update();
    double calc_weight(Particle& p);
    void normalize_weight();
    double likelihood(double x,double mu,double dev);
    double dist_on_map(double map_x,double map_y,double lasor_angle);
    int get_map_occupancy(double x,double y);
    bool reset_request();
    void expansion_reset();
    void resampling();
    void estimate_pose();
    double get_max_weight();
    void reset_weight();

    void publish_particles();
    void publish_estimated_pose();
    void broadcast_roomba_state();

    int hz_;
    int num_;
    double init_x_;
    double init_y_;
    double init_yaw_;
    double init_dev_;
    double motion_dev_per_dist_;
    double motion_dev_per_rot_;
    double laser_dev_per_dist_;
    double expansion_reset_dev_;
    double resampling_reset_dev_;
    int laser_step_;
    double laser_ignore_range_;
    int expansion_limit_;
    double alpha_th_;
    double alpha_slow_th_;
    double alpha_fast_th_;
    bool is_visible_;


    int expansion_count_ = 0;
    double alpha_ = 0.0;
    double alpha_slow_ = 0.0;
    double alpha_fast_ = 0.0;
    int num_replace_ = 0;

    bool init_request_ = true;
    bool odometry_got_ = false;
    bool map_got_ = false;
    bool laser_got_ = false;
    bool can_move_ = false;

    Particle estimated_pose_;
    std::vector<Particle> particles_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber sub_laser_;
    ros::Subscriber sub_odometry_;
    ros::Subscriber sub_map_;

    ros::Publisher pub_estimated_pose_;
    ros::Publisher pub_particle_cloud_;

    tf::TransformBroadcaster roomba_state_broadcaster_;

    geometry_msgs::PoseStamped estimated_pose_msg_;
    geometry_msgs::PoseArray particle_cloud_msg_;

    nav_msgs::Odometry current_odometry_;
    nav_msgs::Odometry previous_odometry_;
    nav_msgs::OccupancyGrid map_;

    sensor_msgs::LaserScan laser_;

};

#endif
