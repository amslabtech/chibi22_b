#ifndef LOCALIZER_H
#define LOCALIZER_H

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LasorScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "tf2/utils.h"
#include "roomba_500driver_meiji/RoombaCtrl.h"

#include <vector>
#include <random>
#include <math.h>

class Particle
{
public:
    Particle();
    geometry_msgs::PoseStamped pose_;
    double weight_;
};

class ParticleFilter
{
public:
    ParticleFilter();
    vector<Particle> particles_;
    void odometryUpdate(const nav_msgs::Odometry odometry);
    void motionUpdate();
    void move(double dx,double dy,double dyaw);
    void measurementUpdate();
private:
    void set(double x,double y,double yaw,double x_cov,double y_cov,double yaw_cov);
    void resampling();
    void weightNormalize();
    void weightReset();
    double angleSubstruct(double angle1, double angle2);
    double optimize_angle(double angle);

    Particle p_;
    Localizer mcl_;

    nav_msgs::Odometry last_odom_;
    nav_msgs::Odometry prev_odom_;
};

class Localizer
{
public:
    Localizer();
    void process();

private:
    void odometry_callback(nav_msgs::Odometry::ConstPtr &msg);
    void map_callback(nav_msgs::OccupancyGrid::ConstPtr &msg);
    void laser_callback(sensor_msgs::LaserScan::ConstPtr &msg);


    int hz_;
    int N_;
    double INIT_X_;
    double INIT_Y_;
    double INIT_YAW_;
    double INIT_X_COV_;
    double INIT_Y_COV_;
    double INIT_YAW_COV_;

    bool first_motionUpdate_ = true;

    ParticleFilter pf_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber laser_sub_;
    ros::Subscriber odometry_sub_;
    ros::Subscriber map_sub_;

    ros::Publisher estimated_pose_pub_;
    ros::Publisher p_cloud_pub_;

    geometry_msgs::PoseStamped estimated_pose_;
    geometry_msgs::PoseStamped poses_;
    nav_msgs::Odometry odometry_;
    nav_msgs::OccupancyGrid map_;
    sensor_msgs::LasorScan laser_;

};

#endif
