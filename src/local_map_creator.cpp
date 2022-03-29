#include "local_map_creator_wakabayashi/local_map_creator.h"

LocalMapCreator::local_map_creator():private_nh_("~")
{
    private_nh_.param("hz", hz_, {1});
}

void LocalMapCreator::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
}
