#include "local_map_creator/local_map_creator.h"

LocalMapCreator::LocalMapCreator():private_nh_("~")
{
    private_nh_.getParam("hz_", hz_);
    private_nh_.getParam("map_size_", map_size_);
    private_nh_.getParam("map_reso_", map_reso_);
    private_nh_.getParam("roomba_radius_", roomba_radius_);

    laser_sub_ = nh_.subscribe("scan", 10, &LocalMapCreator::laser_callback, this);
    local_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("local_map", 1);
    obstacle_poses_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/local_map/obstacle", 1);

    obstacle_poses_.header.frame_id = "base_link";
    local_map_.header.frame_id = "base_link";
    local_map_.info.resolution = map_reso_;
    local_map_.info.width = map_size_ * map_reso_;
    local_map_.info.height = map_size_ * map_reso_;
    local_map_.info.origin.position.x = - (map_size_ * map_reso_) / 2;
    local_map_.info.origin.position.y = - (map_size_ * map_reso_) / 2;
    local_map_.data.reserve(local_map_.info.width * local_map_.info.height);
    init_map();
}

void LocalMapCreator::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laser_ = *msg;
    is_laser_checker_ = true;
}

void LocalMapCreator::init_map()
{
    local_map_.data.clear();  //配列の中身を初期化
    int size = local_map_.info.width * local_map_.info.height;
    for(int i=0; i<size; i++) {
        local_map_.data.push_back(-1);
    }
}

int LocalMapCreator::calc_distance(double angle)
{
    double x = 0;
    double y = 0;
    if(angle < 0) {
        x = local_map_.info.origin.position.x;
        y = - x * std::tan(angle);
    } else if((angle >= 0) && (angle < M_PI / 4)) {
        x = local_map_.info.origin.position.x;
        y = x * std::tan(angle);
    } else if((angle >= M_PI / 4) && (angle < M_PI / 2)) {
        y = - local_map_.info.origin.position.y;
        x = y / std::tan(angle);
    } else if((angle >= M_PI / 2) && (angle < 3 * M_PI / 4)) {
        y = - local_map_.info.origin.position.y;
        x = - y / std::tan(angle);
    } else if((angle >= 3 * M_PI / 4) && (angle < M_PI)) {
        x = - local_map_.info.origin.position.x;
        y = - x * std::tan(angle);
    } else if((angle >= M_PI) && (angle <= 5 * M_PI / 4)) {
        x = - local_map_.info.origin.position.x;
        y = x * std::tan(angle);
    }
    return hypot(x, y);
}

bool LocalMapCreator::is_map_range_checker(double x, double y)
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

bool LocalMapCreator::is_ignore_angle_checker(double angle)
{
    if(angle < - 3/16 * M_PI) {
        return false;
    } else if((angle > 3/16 * M_PI) && (angle < 5/16 * M_PI )) {
        return false;
    } else if((angle > 11/16 * M_PI) && (angle < 13/16 * M_PI)) {
        return false;
    } else if(angle > 19/16 * M_PI) {
        return false;
    } else {
        return true;
    }
}

void LocalMapCreator::create_line(double angle, double laser_range)
{
    double x_now = 0;
    double y_now = 0;

    for(int distance = 0; distance < calc_distance(angle); distance++) {
        x_now = distance * std::cos(angle);
        y_now = distance * std::sin(angle);

        if(!is_map_range_checker(x_now, y_now)) {
            return;
        }

        if(is_ignore_angle_checker(angle)) {
            if(distance >= laser_range) {
                local_map_.data[distance] = 100;
                geometry_msgs::Pose obstacle_pose_;
                obstacle_pose_.position.x = x_now;
                obstacle_pose_.position.y = y_now;
                obstacle_poses_.poses.push_back(obstacle_pose_);
            } else {
                local_map_.data[distance] = 0;
            }
        } else {
            local_map_.data[distance] = 0;
        }
    }
}

void LocalMapCreator::create_local_map()
{
    obstacle_poses_.poses.clear();
    int laser_size = laser_.angle_max - laser_.angle_min;
    int scan_size = laser_size / laser_.angle_increment;
    double angle = 0;

    for(int i=0; i<scan_size; i++) {
        angle = (i * laser_.angle_increment + laser_.angle_min) - (laser_size / 6);
        create_line(angle, laser_.ranges[i]);
    }
}

void LocalMapCreator::process()
{
    ros::Rate loop_rate(hz_);
    while(ros::ok()) {
        if(is_laser_checker_) {
            init_map();
            create_local_map();
            local_map_pub_.publish(local_map_);
            obstacle_poses_pub_.publish(obstacle_poses_);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "local_map_creator");
    LocalMapCreator localmapcreator;
    localmapcreator.process();

    return 0;
}
