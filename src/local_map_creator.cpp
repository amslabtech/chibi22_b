#include "local_map_creator/local_map_creator.h"

LocalMapCreator::LocalMapCreator():private_nh_("~")
{
    private_nh_.param("hz", hz_, {10});
    private_nh_.param("map_size", map_size_, {4});
    private_nh_.param("map_reso", map_reso_, {0.02});
    private_nh_.param("roomba_radius", roomba_radius_, {0.2});

    laser_sub_ = nh_.subscribe("scan", 10, &LocalMapCreator::laser_callback, this);
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
    init_map();
}

void LocalMapCreator::laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{

    // std::cout << "laser in" << std::endl;
    laser_ = *msg;
    init_map();
    create_local_map();
    // std::cout << "laser out" << std::endl;
    is_laser_checker_ = true;
    // std::cout << "laser true" << std::endl;
}

void LocalMapCreator::init_map()
{
    local_map_.data.clear();
    int size = local_map_.info.width * local_map_.info.height;
    for(int i=0; i<size; i++) {
        local_map_.data.push_back(-1);
    }
}

/*int LocalMapCreator::calc_distance(double angle)
{
    double base = local_map_.info.width / 2;
    double tmp = 0;
    angle = abs(angle);

    if((angle >= 0) && (angle < M_PI / 4)) {
        tmp = base * tan(angle)
    } else {
        tmp = base / tan(angle);
    }
    return hypot(base, tmp);
}*/

int LocalMapCreator::xy_to_map_index(double x, double y)
{
    // int x_index = abs(x + local_map_.info.origin.position.x) / map_reso_;
    // int y_index = abs(y + local_map_.info.origin.position.y) / map_reso_;

    int x_index = (x - local_map_.info.origin.position.x) / map_reso_;
    int y_index = (y - local_map_.info.origin.position.y) / map_reso_;
    // std::cout << "x_now : " << x << std::endl;
    // std::cout << "x_index : " << x_index << std::endl;
    // std::cout << "y_now : " << y << std::endl;
    // std::cout << "y_index : " << y_index << std::endl;

    return x_index + y_index * local_map_.info.width;
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
    angle = abs(angle);

    // std::cout << "abs(angle) : " << angle << std::endl;

    if((angle > M_PI * 3/16) && (angle < M_PI * 5/16)) {
        // std::cout << "false" << std::endl;
        return false;
    } else if(angle > M_PI * 11/16) {
        // std::cout << "false" << std::endl;
        return false;
    } else {
        // std::cout << "true" << std::endl;
        return true;
    }
}

bool LocalMapCreator::is_range_checker(double laser_range)
{
    if(laser_range < roomba_radius_) {
        return false;
    } else {
        return true;
    }
}

void LocalMapCreator::create_line(double angle, double laser_range)
{
    if(!is_range_checker(laser_range) || (!is_ignore_angle_checker(angle))) {
        laser_range = map_size_;
    }

    // std::cout << "create_line start" << std::endl;

    for(double distance = 0; distance < map_size_; distance+=map_reso_) {
        // std::cout << "for start" << std::endl;
        double x_now = distance * std::cos(angle);
        double y_now = distance * std::sin(angle);

        if(!is_map_range_checker(x_now, y_now)) {
            return;
        }

        int map_index = xy_to_map_index(x_now, y_now);
        // std::cout << "map_index : " << map_index << std::endl;

        if(distance >= laser_range) {
            // std::cout << "obstacle start" << std::endl;
            // std::cout << "distance : " << distance << std::endl;
            local_map_.data[map_index] = 100;
            obstacle_pose_.position.x = x_now;
            obstacle_pose_.position.y = y_now;
            // std::cout << "x_now : " << x_now << std::endl;
            // std::cout << "y_now : " << y_now << std::endl;
            obstacle_poses_.poses.push_back(obstacle_pose_);
            return;
            // std::cout << "push_back!" << std::endl;
        } else {
            local_map_.data[map_index] = 0;
        }
        // std::cout << "for end" << std::endl;
    }
    // std::cout << "create_line end" << std::endl;
}

void LocalMapCreator::create_local_map()
{
    // std::cout << "create_local_map start" << std::endl;
    obstacle_poses_.poses.clear();
    double angle_size = laser_.angle_max - laser_.angle_min;
    int angle_step = int((laser_.ranges.size() * M_PI) / angle_size / 180 / 2);

    // std::cout << "size" << laser_.ranges.size() << std::endl;
    // std::cout << "angle_size : " << angle_size << std::endl;
    // std::cout << "angle_step : " << angle_step << std::endl;

    for(int i=0; i<int(laser_.ranges.size()); i+=angle_step) {
        double angle = i * laser_.angle_increment + laser_.angle_min;
        // std::cout << "angle :" << angle << std::endl;
        // std::cout << "laser_.ranges :" << laser_.ranges[i] << std::endl;
        create_line(angle, laser_.ranges[i]);
    }
    // std::cout << "create_local_map finish" << std::endl;
}

void LocalMapCreator::process()
{
    ros::Rate loop_rate(hz_);
    while(ros::ok()) {
        if(is_laser_checker_) {
            // init_map();
            // std::cout << "map_size_ :" << map_size_ << std::endl;
            // create_local_map();
            // local_map_pub_.publish(local_map_);
            local_map_pub_.publish(local_map_);
            obstacle_poses_pub_.publish(obstacle_poses_);
            // std::cout << "publish!" << std::endl;
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
