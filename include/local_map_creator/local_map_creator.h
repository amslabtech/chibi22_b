#ifndef local_map_cretor_H
#define local_map_cretor_H

#include "ros/ros.h"
#include "roomba_500driver_meiji/RoombaCtrl.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseArray.h"
#include "tf/tf.h"

class LocalMapCreator
{
    public:
        local_map_creator();
        void process();
    private:
        void laser_callback(const sensor_msgs::LaserScan::ConctPtr&);
        void odometry_callback(const nav_msgs::Odometry::ConstPtr&);
        void create_line(double yaw, double laser_range);
        void create_local_map();
        void init_map();
        int xy_to_map_index(double x, double y);
        float get_yaw();
        bool is_map_range_checker(double x, double y);
        bool is_ignore_angle_checker(double angle);

        int hz;
        int map_size;
        double map_reso;
        double laser_density;
        double roomba_radius;
        double ignore_angle_mergin;

        bool is_laser_checker = false;
        bool is_odometry_checker = false;

        ros::NodeHandle nh;
        ros::NodeHandle private_nh;

        ros::Subscriber laser_sub;
        ros::Subscriber odometry_sub;
        ros::Publisher local_map_pub;
        ros::Publisher obstacle_poses_pub;

        sensor_msgs::LaserScan laser;
        nav_msgs::Odometry odometry;
        nav_msgs::OccupancyGrid local_map;
        geometry_msgs::PoseArray obstacle_poses;
};

#endif
