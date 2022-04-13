#include "local_map_creator_wakabayashi/local_map_creator.h"

LocalMapCreator::local_map_creator():private_nh("~")
{
    private_nh.getparam("hz_", hz_);
    private_nh.getparam("map_size_", map_size_);
    private_nh.getparam("map_reso_", map_reso_);
    private_nh.getparam("roomba_radius_", roomba_radius_);

    laser_sub = nh.subscribe("scan", 10, &LocalMapCreator::laser_callback, this);
    local_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("local_map", 10);
    obstacle_poses_pub = nh.advertise<geometry_msgs::PoseArray>("obstacle_poses", 10);

    obstacle_poses.header.frame_id = "base_link";
    local_map.header.frame_id = "base_link";
    local_map.info.resolution = map_reso;
    local_map.info.width = map_size_ / map_reso_;
    local_map.info.height = map_size_ / map_reso_;
    local_map.info.origin.position.x = 0;
    local_map.info.origin.position.y = 0;
    local_map.data.reserve(local_map.info.width * local_map.info.height);
    init_map();
}

void LocalMapCreator::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laser = *msg;
    is_laser_checker = true;
}

void LocalMapCreator::init_map()
{
    local_map.data.clear();  //配列の中身を初期化
    int size = local_map.info.width * local_map.info.height;
    for(int i=0; i<size; i++) {
        local_map.data.push_back(-1);
    }
}

int LocalMapCreator::xy_to_map_index(double x, double y)
{
    int index_x = int((x - local_map.info.origin.position.x) / local_map.info.resolution);
    int index_y = int((y - local_map.info.origin.position.y) / local_map.info.resolution);

    return index_x + index_y;
}

bool LocalMapCreator::is_map_range_checker(double x, double y)
{
    double x_start = local_map.info.origin.position.x;
    double y_start = local_map.info.origin.position.y;
    double x_end = x_start + local_map.info.width * local_map.info.resolution;
    double y_end = y_start + local_map.info.height * local_map.info.resolution;

    if((x_start < x) && (x_end > x) && (y_start < y) && (y_end > y)) {
        return true;
    } else {
        return false;
    }
}

bool LocalMapCreator::is_ignore_angle_checker(double angle)
{
    if(angle < 1/16 * M_PI) {
        return false;
    } else if((angle > 7/16 * M_PI) && (angle < 9/16 * M_PI )) {
        return false;
    } else if((angle > 15/16 * M_PI) && (angle < 17/16 * M_PI)) {
        return false;
    } else if(angle > 23/16 * M_PI) {
        return false;
    } else {
        return true;
    }
}

void LocalMapCreator::create_line(double angle, double laser_range)
{
    if(laser_range <= roomba_radius) {
            laser_range = map_size;
    }

    double x_now = 0;
    double y_now = 0;
    int map_index = xy_to_map_index(x_now, y_now);

    for(double dist_from_start = 0; dist_from_start < map_size; dist_from_start += map_reso) {
        x_now = dist_from_start * std::cos(angle);
        y_now = dist_from_start * std::sin(angle);

        if(is_map_range_checker(x_now, y_now) = false) {
            return;
        }
        if(is_ignore_angle_checker(angle) = true) {
            if(dist_from_start >= laser_range) {
                local_map.data[map_index] = 100;
                geometry_msgs::Pose obstacle_pose;
                obstacle_pose.position.x = x_now;
                obstacle_pose.position.y = y_now;
                obstacle_poses.poses.push_back(obstacle_pose);
            } else {
                local_map.data[map_index] = 0;
            }
        } else {
            local_map.data[map_index] = 0;
        }
    }
}

void LocalMapCreator::create_local_map()
{
    obstacle_poses.poses.clear();
    int scan_size = laser.ranges.size();
    int scan_step = (laser.angle_max - laser.angle_min) / scan_size;
    double angle = 0;

    for(int i=0; i<scan_size; i+=scan_step) {
        angle = i * laser.angle_increment + laser.angle_min;
        create_line(angle, laser.ranges[i]);
    }
}

void LocalMapCreator::process()
{
    ros::Rate rate(hz_);
    while(ros::ok()) {
        if(is_laser_checker = true) {
            init_map();
            create_local_map();
            local_map_pub.publish(local_map);
            obstacle_poses_pub.publish(obstacle_poses);
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
