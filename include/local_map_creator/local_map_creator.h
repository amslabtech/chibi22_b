#ifndef local_map_cretor_H
#define local_map_cretor_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseArray.h"

class LocalMapCreator
{
    public:
        local_map_creator();
        void process();

    private:
        void laser_callback(const sensor_msgs::LaserScan::ConctPtr&);



        sensor_msgs::LaserScan::laser_;
        nav_msgs::OccupancyGrid::local_map;
        geometry_msgs::PoseArray obstacle_poses;

