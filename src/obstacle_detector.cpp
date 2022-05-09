#include "obstacle_detector/obstacle_detector.h"

ObstacleDetector::ObstacleDetector():private_nh_("~")
{
    private_nh_.param("hz", hz_, {10});
    private_nh_.param("map_size", map_size_, {4});
    private_nh_.param("map_reso", map_reso_, {0.02});
    private_nh_.param("roomba_radius", roomba_radius_, {0.2});
    private_nh_.param("flag_map_view", flag_map_view_, {false});

    laser_sub_ = nh_.subscribe("scan", 10, &ObstacleDetector::laser_callback, this);
    local_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("local_map", 10);
    obstacle_poses_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/local_map/obstacle", 10);

    obstacle_poses_.header.frame_id = "base_link";
    local_map_.header.frame_id = "base_link";
    local_map_.info.resolution = map_reso_;
    local_map_.info.width = map_size_ / map_reso_;
    local_map_.info.height = map_size_ / map_reso_;
    local_map_.info.origin.position.x = - map_size_ / 2;
    local_map_.info.origin.position.y = - map_size_ / 2;
    local_map_.data.reserve(local_map_.info.width * local_map_.info.height);
    // init_map();
}

void ObstacleDetector::laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{

    laser_ = *msg;
    if(flag_map_view_) {
        init_map();
    }
    create_local_map();
    is_laser_checker_ = true;
}

void ObstacleDetector::init_map()
{
    local_map_.data.clear();
    int size = local_map_.info.width * local_map_.info.height;
    for(int i=0; i<size; i++) {
        local_map_.data.push_back(-1);
    }
}

int ObstacleDetector::xy_to_map_index(double x, double y)
{
    int x_index = (x - local_map_.info.origin.position.x) / map_reso_;
    int y_index = (y - local_map_.info.origin.position.y) / map_reso_;

    return x_index + y_index * local_map_.info.width;
}

bool ObstacleDetector::is_map_range_checker(double x, double y)
{
    double x_min = local_map_.info.origin.position.x;
    double y_min = local_map_.info.origin.position.y;
    double x_max = x_min + local_map_.info.width * local_map_.info.resolution;
    double y_max = y_min + local_map_.info.height * local_map_.info.resolution;

    if((x_min < x) && (x_max > x) && (y_min < y) && (y_max > y)) {
        return true;
    } else {
        return false;
    }
}

bool ObstacleDetector::is_ignore_angle_checker(double angle)
{
    // left
    if((angle < M_PI * - 1.2/16) && (angle > M_PI * - 5/16))
    {
        return false;
    }
    // right
    if((angle > M_PI * 3/16) && (angle < M_PI * 5/16))
    {
        return false;
    }

    angle = abs(angle);
    if(angle > M_PI * 11/16) {
        return false;
    } else {
        return true;
    }


    // angle = abs(angle);
    // if((angle > M_PI * 2/16) && (angle < M_PI * 5/16)) {
    //     return false;
    // } else if(angle > M_PI * 11/16) {
    //     return false;
    // } else {
    //     return true;
    // }
}

bool ObstacleDetector::is_range_checker(double laser_range)
{
    if(laser_range < roomba_radius_) {
        return false;
    } else {
        return true;
    }
}

void ObstacleDetector::create_line(double angle, double laser_range)
{
    // if(!is_range_checker(laser_range) || (!is_ignore_angle_checker(angle))) {
    //     laser_range = map_size_;
    // }
    if(!is_ignore_angle_checker(angle)) {
        laser_range = map_size_;
    }

    for(double distance = 0; distance < map_size_; distance+=map_reso_) {
        double x_now = distance * std::cos(angle);
        double y_now = distance * std::sin(angle);

        if(!is_map_range_checker(x_now, y_now)) {
            return;
        }

        int map_index = xy_to_map_index(x_now, y_now);

        if(distance >= laser_range) {
            if(flag_map_view_) {
                local_map_.data[map_index] = 100;
            }
            obstacle_pose_.position.x = x_now;
            obstacle_pose_.position.y = y_now;
            obstacle_poses_.poses.push_back(obstacle_pose_);
            return;
        } else {
            if(flag_map_view_) {
                local_map_.data[map_index] = 0;
            }
        }
    }
}

void ObstacleDetector::create_local_map()
{
    obstacle_poses_.poses.clear();
    double angle_size = laser_.angle_max - laser_.angle_min;
    int angle_step = int((laser_.ranges.size() * M_PI) / angle_size / 180 / 2);

    for(int i=0; i<int(laser_.ranges.size()); i+=angle_step) {
        double angle = i * laser_.angle_increment + laser_.angle_min;
        create_line(angle, laser_.ranges[i]);
    }
}

void ObstacleDetector::process()
{
    ros::Rate loop_rate(hz_);
    while(ros::ok()) {
        if(is_laser_checker_) {
            if(flag_map_view_) {
                local_map_pub_.publish(local_map_);
            }
            obstacle_poses_pub_.publish(obstacle_poses_);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "local_map_creator");
    ObstacleDetector obstacledetector;
    obstacledetector.process();

    return 0;
}