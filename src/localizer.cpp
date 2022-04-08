#include "localizer/localizer.h"

std::random_device seed;
std::mt19937 engine(seed());

Particle::Particle(double x, double y, double yaw, double weight)
{
    set(x,y,yaw,weight);
}

void Particle::set(double x, double y, double yaw, double weight)
{
    x_ = x;
    y_ = y;
    yaw_ = yaw;
    weight_ = weight;
}

ParticleFilter::ParticleFilter()
{
    N_ = mcl_ -> get_N();
    init_x_ = mcl_ -> get_INIT_X();
    init_y_ = mcl_ -> get_INIT_Y();
    init_yaw_ = mcl_ -> get_INIT_YAW();
    x_cov_ = mcl_ -> get_INIT_X_COV();
    y_cov_ = mcl_ -> get_INIT_Y_COV();
    yaw_cov_ = mcl_ -> get_INIT_YAW_COV();
}

void ParticleFilter::initialize()
{
    check_N();
    double w = 1.0/(double)N_;

    for(int i=0; i<N_; i++){
        double x = set_noise(init_x_,x_cov_);
        double y = set_noise(init_y_,y_cov_);
        double yaw = set_noise(init_yaw_,yaw_cov_);
        Particle p(x,y,yaw);
        particles_.push_back(p);
    }
}

void ParticleFilter::motion_update(nav_msgs::Odometry last, nav_msgs::Odometry prev)
{
    double dx = last.pose.pose.position.x - prev.pose.pose.position.x;
    double dy = last.pose.pose.position.y - prev.pose.pose.position.y;
    double last_yaw = tf2::getYaw(last.pose.orientation);
    double prev_yaw = tf2::getYaw(prev.pose.orientation);
    double dyaw = optimize_angle(last_yaw - prev_yaw);

    double distance = sqrt(dx*dx + dy*dy);
    double direction = optimize_angle(atan2(dy,dx) - prev_yaw);

    for(auto& p:particles_)
        p.move(distance, direction, dyaw);
}

void ParticleFilter::move(double distance, double direction, double rotation)
{
    distance = set_noise(distance, distance_cov_);
    direction = set_noise(direction, rotation_cov_);
    rotation = set_noise(rotation, rotation_cov_);

    new_x = get_pose_x() + distance * cos( optimize_angle(direction + getPose_yaw()) );
    new_y = get_pose_y() + distance * sin( optimize_angle(direction + getPose_yaw()) );
    new_yaw = optimize_angle(getPose_yaw() + rotation);

    set(new_x, new_y, new_yaw, get_weight());
}

double set_noise(double mu, double cov)
{
    std::normal_distribution<> dist(mu,cov);
    return dist(engine)
}

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

    laser_sub_ = nh.subscribe("/scan", 10, &Localizer::laser_callback, this);
    odometry_sub_ = nh.subscribe("/roomba/odometry", 10, &Localizer::odometry_callback, this);
    map_sub_ = nh.subscribe("/map", 10, &Localizer::map_callback, this);

    estimated_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/estimated_pose", 1);
    p_cloud_pub_ = nh.advertise<geometry_msgs::PoseArray>("/p_cloud", 1);

    estimated_pose.flame_id = "map";
    p_cloud.header.flame_id = "map";
}

void Localizer::odometry_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    odometry_ = *msg;
}

void Localizer::map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    map_ = *msg;
}

void Localizer::laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    laser_ = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "localizer");
    Localizer localizer;
    localizer.process();

    return 0;
}
