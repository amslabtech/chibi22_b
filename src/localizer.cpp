#include<Localizer/Localizer.h>

std::random_device seed;
std::mt19937 engine(seed());

Particle::Particle(double x, double y, double yaw, double weight)
{
    set(x,y,yaw,weight);
}

Particle::set(double x, double y, double yaw, double weight)
{
    x_ = x;
    y_ = y;
    yaw_ = yaw;
    weight_ = weight;
}

ParticleFilter::ParticleFilter()
{
    N_ = mcl_ -> getN();
    init_x_ = mcl_ -> getINIT_X();
    init_y_ = mcl_ -> getINIT_Y();
    init_yaw_ = mcl_ -> getINIT_YAW();
    x_cov_ = mcl_ -> getINIT_X_COV();
    y_cov_ = mcl_ -> getINIT_Y_COV();
    yaw_cov_ = mcl_ -> getINIT_YAW_COV();
}

ParticleFilter::initialize()
{
    check_N();
    double w = 1.0/(double)N_;

    std::vector<Particle> init_particles;
    for(int i=0; i<N_; i++){
        double x = noiseAdd(init_x_,x_cov_);
        double y = noiseAdd(init_y_,y_cov_);
        double yaw = noiseAdd(init_yaw_,yaw_cov_);
        pParticle_ = new Particle(x,y,yaw,w);
        init_particles.push_back(pParticle_);


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
