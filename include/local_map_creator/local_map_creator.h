#ifndef local_map_creator_H
#define local_map_creator_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"

class LocalMapCreator
{
    public:
        LocalMapCreator();
        void process();
    private:
        void laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg);
        void create_line(double angle, double laser_range);
        void create_local_map();
        void init_map();
        int calc_distance(double angle);
        int xy_to_map_index(double x, double y);
        bool is_map_range_checker(double x, double y);
        bool is_ignore_angle_checker(double angle);
        bool is_range_checker(double laser_range);

        int hz_;
        double map_size_;
        double map_reso_;
        double roomba_radius_;

        bool is_laser_checker_ = false;

        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        ros::Subscriber laser_sub_;
        ros::Publisher local_map_pub_;
        ros::Publisher obstacle_poses_pub_;

        sensor_msgs::LaserScan laser_;
        nav_msgs::OccupancyGrid local_map_;
        geometry_msgs::PoseArray obstacle_poses_;
        geometry_msgs::Pose obstacle_pose_;
};

#endif
