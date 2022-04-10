#include "local_map_creator_wakabayashi/local_map_creator.h"

LocalMapCreator::local_map_creator():private_nh("~")
{
    private_nh.getparam("hz", hz);
    private_nh.getparam("map_size", map_size);
    private_nh.getparam("map_reso", map_reso);
    private_nh.getparam("laser_density", laser_density);
    private_nh.getparam("roomba_radius", roomba_radius);
    private_nh.getparam("ignore_angle_mergin", ignore_angle_mergin);

    laser_sub = nh.subscribe("scan", 10, &LocalMapCreator::laser_callback, this);
    local_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("local_map", 10);
    obstacle_poses_pub = nh.advertise<geometry_msgs::PoseArray>("obstacle_poses", 10);
    obstacle_poses.header.frame_id = "base_link";
    local_map.header.frame_id = "base_link";
    local_map.info.resolution = map_reso;
    local_map.info.width = map_size / map_reso;
    local_map.info.height = map_size / map_reso;
    local_map.info.origin.position.x = - map_size / 2;
    local_map.info.origin.position.y = - map_size / 2;
    local_map.data.reserve(local_map.info.width * local_map.info.height);
    init_map();
}

void LocalMapCreator::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laser = *msg;
    init_map();
    create_local_map();
    laser_get_check = true;
}

void LocalMapCreator::init_map()
{
    local_map.data.clear();  //配列の中身を初期化
    for(int i=0; size=local_map.info.width * local_map.info.height; i<size; i++) {
        local_map.data.push_back(-1);
    }
}

int LocalMapCreator::xy_to_map_index(double x, double y)
{
    int index_x = int((x - local_map.info.origin.position.x) / local_map.info.resolution);
    int index_y = int((y - local_map.info.origin.position.y) / local_map.info.resolution);

    return index_x + index_y * local_map.info.width;
}

bool LocalMapCreator::check_map_range(double x, double y)
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

bool LocalMapCreator::is_ignore_angle(double angle)
{
    if((angle > -3.0/4 * M_PI + ignore_angle_mergin) && (angle < -1.0/4 * M_PI - ignore_angle_mergin)) {
        return false;
    } else if((angle > -1.0/4 * M_PI + ignore_angle_mergin) && (angle < 1.0/4 * M_PI - ignore_angle_mergin)) {
        return false;
    } else if((angle > 1.0/4 * M_PI + ignore_angle_mergin) && (angle < 3.0/4 * M_PI - ignore_angle_mergin)) {
        return false;
    }
    return true;
}

void LocalMapCreator::create_line(double yaw, double laser_range)
{
    double search_step = map_reso;

    if((laser_range <= roomba_radius) || (is_ignore_angle(yaw))) {
            laser_range = map_size;
    }

    double x_now = 0;
    double y_now = 0;

    for(double dist_from_start = 0; dist_from_start < map_size; dist_from_start += search_step) {
        x_now = dist_from_start * std::cos(yaw);
        y_now = dist_from_start * std::sin(yaw);

        if(!check_map_range(x_now, y_now)) {
            return;
        }

        int map_index = xy_to_map_index(x_now, y_now);

        if(dist_from_start >= laser_range) {
            local_map.data[map_index] = 100;
            geometry_msgs::Pose obstacle_pose;
            obstacle_pose.position.x = x_now;
            obstacle_pose.position.y = y_now;
            obstacle_poses.poses.push_back(obstacle_pose);
            return;
        } else {
            local_map.data[map_index] = 0;
        }
    }
}

void LocalMapCreator::create_local_map()
{
    obstacle_poses.poses.clear();
    double scan_angle = laser.angle_max - laser.angle_min;
    int laser_step = int(2 * map_reso * laser.ranges.size() / laser_density / scan_angle / map_size);
    double angle = 0;

    for(int i=0; i<int(laser.ranges.size()) i+=laser_step) {
        angle = i * laser.angle_increment + laser.angle_min;
        create_line(angle, laser.ranges[i]);
    }
}

void LocalMapCreator::process()
{
    ros::Rate rate(hz);
    while(ros::ok()) {
        if(laser_get_check) {
            local_map_pub.publish(local_map);
            obstacle_poses_pub.publish(obstacle_poses);
        }
        ros::spinOnce();
        rate::sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "local_map_creator");
    LocalMapCreator localmapcreator;
    localmapcreator.process();

    return 0;
}
