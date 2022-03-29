#include<Localizer/Localizer.h>

std::random_device seed;
std::mt19937 engine(seed());

ParticleFilter::ParticleFilter()
{
    Particle p();

Localizer::Localizer():private_nh("~")
{
    private_nh.getParam("hz",hz_);
    private_nh.getParam("N",N_);
    private_nh.getParam("INIT_X",INIT_X_);
    private_nh.getParam("INIT_Y",INIT_Y_);
    private_nh.getParam("INIT_YAW",INIT_YAW_);
    private_nh.getParam("INIT_X_COV",INIT_X_COV_);
    private_nh.getParam("INIT_Y_COV",INIT_Y_COV_);
    private_nh.getParam("INIT_YAW_COV",INIT_YAW_COV_);

    laser_sub_ = nh.subscribe("/scan", 100, &Localizer::laser_callback, this);
    odometry_sub_ = nh.subscribe("/roomba/odometry", 100, &Localizer::odometry_callback, this);
    map_sub_ = nh.subscribe("/map", 100, &Localizer::map_callback, this);

    estimated_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/estimated_pose", 100);
    p_cloud_pub_ = nh.advertise<geometry_msgs::PoseArray>("/p_cloud", 100);

    estimated_pose.flame_id = "map";
    p_cloud.header.flame_id = "map";
}

void Localizer::odometry_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    odometry_ = *msg;
    pf_.odometryUpdate(odometry_);
}

void Localizer::map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    map_ = *msg;
}

void Localizer::laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    laser_ = *msg;
}

double Localizer::add_noise(double mu, double sigma)
{
    std::normal_distribution<> dist(mu, sigma);
    return dist(engine);
}

void Localizer::p_init(double init_x, double init_y, double init_yaw, double init_x_cov, double init_y_cov, double init_yaw_cov)
{
    pose.pose.position.x = add_noise(init_x, init_x_cov);
    pose.pose.position.y = add_noise(init_y, init_y_cov);
    double yaw = add_noise(init_yaw, init_yaw_cov);
    quaternionTFToMsg(tf::createQuaternionFromYaw(yaw), pose.pose.orientation);
}

void Localizer::p_motionUpdate()
{
    if(first_motionUpdate_){
        prev_odom_ = last_odom_;
        first_motionUpdate_ = false;
        return;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "localizer");
    Localizer localizer;
    localizer.process();

    return 0;
}
