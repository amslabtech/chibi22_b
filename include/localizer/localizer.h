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
    Particle(double x=0,double y=0,double yaw=0,double weight=0);
    void set(double x,double y,double yaw,double weight);
    double getPose_x(){return x_;}
    double getPose_y(){return y_;}
    double getPose_yaw(){return yaw_;}
    double getWeight(){return weight_;}
private:
    double x_;
    double y_;
    double yaw_;
    double weight_;
};

class ParticleFilter
{
public:
    ParticleFilter();
    void initialize();
    void motionUpdate(nav_msgs::Odometry last,nav_msgs::Odometry prev);
    void move(double distance,double direction,double rotation);
    void measurementUpdate();
    void resampling();
    void check_N();

    std::vector<Particle> particles_;

private:
    double setNoise(double mu,double cov);
    double angleOptimize(double angle);
    void weightNormalize();

    double N_;
    double init_x_;
    double init_y_;
    double init_yaw_;
    double x_cov_;
    double y_cov_;
    double yaw_cov_;

    Localizer* pMcl_;
};

class Localizer
{
public:
    Localizer();
    void process();

    double getN(){return N_;}
    double getINIT_X(){return INIT_X_;}
    double getINIT_Y(){return INIT_Y_;}
    double getINIT_YAW(){return INIT_YAW_;}
    double getINIT_X_COV(){return INIT_X_COV_;}
    double getINIT_Y_COV(){return INIT_Y_COV_;}
    double getINIT_YAW_COV(){return INIT_YAW_COV_;}

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

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber laser_sub_;
    ros::Subscriber odometry_sub_;
    ros::Subscriber map_sub_;

    ros::Publisher estimated_pose_pub_;
    ros::Publisher p_cloud_pub_;

    geometry_msgs::PoseStamped estimated_pose_;
    geometry_msgs::PoseStamped poses_;
    nav_msgs::Odometry last_odometry_;
    nav_msgs::Odometry prev_odometry_;
    nav_msgs::OccupancyGrid map_;
    sensor_msgs::LasorScan laser_;

};

#endif
