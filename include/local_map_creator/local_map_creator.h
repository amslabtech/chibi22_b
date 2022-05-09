#ifndef local_map_creator_H
#define local_map_creator_H

#include "ros/ros.h"
#include "tf2/utils.h"
#include "roomba_500driver_meiji/RoombaCtrl.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"

class LocalMapCreator
{
    public:
        LocalMapCreator();
        void process();
    private:
        void laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg);
        void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void odo_callback(const nav_msgs::Odometry::ConstPtr &msg);
        void create_line(double angle, double laser_range, bool first_map_checker_);
        void create_local_map();
        void init_map();
        int xy_to_map_index(double x, double y);
        bool is_map_range_checker(double x, double y);
        bool is_ignore_angle_checker(double angle);
        bool is_range_checker(double laser_range);

        int hz_;
        double map_size_;
        double map_reso_;
        double roomba_radius_;

        bool flag_map_view_;
        bool flag_pose_callback_;
        bool flag_odo_callback_;

        // bool flag_map_view_ = false;
        // bool flag_pose_callback_ = false;
        // bool flag_odo_callback_ = false;
        bool is_laser_checker_ = false;
        bool is_first_pose_checker_ = false;
        bool is_second_pose_checker_ = false;
        bool first_map_checker_ = false;

        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        ros::Subscriber laser_sub_;
        ros::Subscriber pose_sub_;
        ros::Subscriber odo_sub_;
        ros::Publisher local_map_pub_;
        ros::Publisher obstacle_poses_pub_;

        sensor_msgs::LaserScan laser_;
        geometry_msgs::PoseStamped current_pose_;
        nav_msgs::Odometry current_odo_;
        geometry_msgs::Pose previous_pose_;
        nav_msgs::Odometry previous_odo_;
        geometry_msgs::Pose diff_;
        nav_msgs::OccupancyGrid local_map_;
        geometry_msgs::PoseArray obstacle_poses_;
        geometry_msgs::PoseArray blind_obstacle_poses_;
        geometry_msgs::Pose obstacle_pose_;
        geometry_msgs::Pose blind_obstacle_pose_;
};

#endif
