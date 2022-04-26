#include "local_map_creator/local_map_creator.h"

LocalMapCreator::LocalMapCreator():private_nh_("~")
{
    private_nh_.param("hz", hz_, {10});
    private_nh_.param("map_size", map_size_, {4});
    private_nh_.param("map_reso", map_reso_, {0.02});
    private_nh_.param("roomba_radius", roomba_radius_, {0.2});

    laser_sub_ = nh_.subscribe("scan", 10, &LocalMapCreator::laser_callback, this);
    pose_sub_ = nh_.subscribe("/estimated_pose", 10, &LocalMapCreator::pose_callback, this);
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
    previous_pose_.position.x = 0;
    previous_pose_.position.y = 0;
    init_map();
}

void LocalMapCreator::laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{

    laser_ = *msg;
    init_map();
    create_local_map();
    is_laser_checker_ = true;
}

void LocalMapCreator::pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_pose_ = *msg;
    if(first_map_checker_) {
        diff_.position.x = current_pose_.pose.position.x - previous_pose_.position.x;
        diff_.position.y = current_pose_.pose.position.y - previous_pose_.position.y;
        previous_pose_.position.x = current_pose_.pose.position.x;
        previous_pose_.position.y = current_pose_.pose.position.y;
    } else {
        diff_.position.x = 0;
        diff_.position.y = 0;
        previous_pose_.position.x = current_pose_.pose.position.x;
        previous_pose_.position.y = current_pose_.pose.position.y;
    }

}

void LocalMapCreator::init_map()
{
    local_map_.data.clear();
    int size = local_map_.info.width * local_map_.info.height;
    for(int i=0; i<size; i++) {
        local_map_.data.push_back(-1);
    }
}

int LocalMapCreator::xy_to_map_index(double x, double y)
{
    int x_index = (x - local_map_.info.origin.position.x) / map_reso_;
    int y_index = (y - local_map_.info.origin.position.y) / map_reso_;

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

    if((angle > M_PI * 3/16) && (angle < M_PI * 5/16)) {
        return false;
    } else if(angle > M_PI * 11/16) {
        return false;
    } else {
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

void LocalMapCreator::create_line(double angle, double laser_range, bool first_map_checker_)
{
    if(!first_map_checker_) {
        if(!is_range_checker(laser_range) || !is_ignore_angle_checker(angle)) {
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
                local_map_.data[map_index] = 100;
                obstacle_pose_.position.x = x_now;
                obstacle_pose_.position.y = y_now;
                obstacle_poses_.poses.push_back(obstacle_pose_);
                return;
            } else {
                local_map_.data[map_index] = 0;
            }
        }
    } else {
        if(!is_range_checker(laser_range) || !is_ignore_angle_checker(angle)) {
            laser_range = map_size_;
        }

        for(double distance = 0; distance < map_size_; distance+=map_reso_) {
            double x_now = distance * std::cos(angle);
            double y_now = distance * std::sin(angle);

            if(!is_map_range_checker(x_now, y_now)) {
                return;
            }

            int map_index = xy_to_map_index(x_now, y_now);

            for(const auto& b_ob_pose : blind_obstacle_poses_.poses) {
                double new_x_now = b_ob_pose.position.x;
                double new_y_now = b_ob_pose.position.y;
                int new_map_index = xy_to_map_index(new_x_now, new_y_now);

                if(map_index == new_map_index) {
                    laser_range = hypot(new_x_now, new_y_now);
                }
            }


            if(distance >= laser_range) {
                local_map_.data[map_index] = 100;
                obstacle_pose_.position.x = x_now;
                obstacle_pose_.position.y = y_now;
                obstacle_poses_.poses.push_back(obstacle_pose_);
                return;
            } else {
                local_map_.data[map_index] = 0;
            }
        }
    }
}

void LocalMapCreator::create_local_map()
{
    if(first_map_checker_) {
        for(const auto& ob_poses : obstacle_poses_.poses) {
            blind_obstacle_pose_.position.x = ob_poses.position.x - diff_.position.x;
            blind_obstacle_pose_.position.y = ob_poses.position.y - diff_.position.y;

            if(!is_ignore_angle_checker(atan2(blind_obstacle_pose_.position.y, blind_obstacle_pose_.position.x))) {
                blind_obstacle_poses_.poses.push_back(blind_obstacle_pose_);
            }
        }
    }

    obstacle_poses_.poses.clear();
    double angle_size = laser_.angle_max - laser_.angle_min;
    int angle_step = int((laser_.ranges.size() * M_PI) / angle_size / 180 / 2);

    for(int i=0; i<int(laser_.ranges.size()); i+=angle_step) {
        double angle = i * laser_.angle_increment + laser_.angle_min;
        create_line(angle, laser_.ranges[i], first_map_checker_);
    }
    blind_obstacle_poses_.poses.clear();
}

void LocalMapCreator::process()
{
    ros::Rate loop_rate(hz_);
    while(ros::ok()) {
        if(is_laser_checker_) {
            local_map_pub_.publish(local_map_);
            obstacle_poses_pub_.publish(obstacle_poses_);
            first_map_checker_ = true;
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
