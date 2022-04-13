#include "localizer/localizer.h"

// 疑似乱数生成器
std::random_device seed_gen;
std::mt19937 engine(seed_gen());

// ==== クラス Particle ====
// コンストラクタ
Particle::Particle(double x, double y, double yaw, double weight)
{
    set_pose(x,y,yaw);
    set_weight(weight);
}

// pose の設定
void Particle::set_pose(double x, double y, double yaw)
{
    x_ = x;
    y_ = y;
    yaw_ = yaw;
}

// 重みの設定
void Particle::set_weight(double weight)
{
    weight_ = weight;
}

// ==== クラス ParticleFilter ====
// コンストラクタ(Localizer のメンバ変数の値を取得)
ParticleFilter::ParticleFilter()
{
    N_ = pMcl_ -> get_N();
    init_x_ = pMcl_ -> get_INIT_X();
    init_y_ = pMcl_ -> get_INIT_Y();
    init_yaw_ = pMcl_ -> get_INIT_YAW();
    x_cov_ = pMcl_ -> get_INIT_X_COV();
    y_cov_ = pMcl_ -> get_INIT_Y_COV();
    yaw_cov_ = pMcl_ -> get_INIT_YAW_COV();
}

// 初期化処理
void ParticleFilter::initialize()
{
    double w = 1.0/(double)N_;

    for(int i=0; i<N_; i++){
        double x = set_noise(init_x_,x_cov_);
        double y = set_noise(init_y_,y_cov_);
        double yaw = set_noise(init_yaw_,yaw_cov_);
        Particle p(x, y, yaw, w);
        particles_.push_back(p);
    }
}

// 移動量の算出
void ParticleFilter::motion_update(nav_msgs::Odometry last, nav_msgs::Odometry prev)
{
    double dx = last.pose.pose.position.x - prev.pose.pose.position.x;
    double dy = last.pose.pose.position.y - prev.pose.pose.position.y;
    double last_yaw = tf2::getYaw(last.pose.orientation);
    double prev_yaw = tf2::getYaw(prev.pose.orientation);
    double dyaw = optimize_angle(last_yaw - prev_yaw);

    double distance = hypot(dx, dy);
    double direction = optimize_angle(atan2(dy,dx) - prev_yaw);

    for(auto& p:particles_)
        move(p, distance, direction, dyaw);
}

// パーティクルの移動
void ParticleFilter::move(Particle& p, double distance, double direction, double rotation)
{
    distance = set_noise(distance, distance_cov_);
    direction = set_noise(direction, rotation_cov_);
    rotation = set_noise(rotation, rotation_cov_);

    new_x = p.get_pose_x() + distance * cos( optimize_angle(direction + p.get_pose_yaw()) );
    new_y = p.get_pose_y() + distance * sin( optimize_angle(direction + p.get_pose_yaw()) );
    new_yaw = optimize_angle(p.get_pose_yaw() + rotation);

    p.set(new_x, new_y, new_yaw, p.get_weight());
}

// ノイズを乗せた値を返す関数
double ParticleFilter::set_noise(double mu, double cov)
{
    std::normal_distribution<> dist(mu,cov);
    return dist(engine);
}

// 適切な角度 (-π ~ π) を返す
void ParticleFilter::optimize_angle(angle)
{
    if(angle > M_PI)
        angle -= 2*M_PI;
    if(angle < -M_PI)
        angle += 2*M_PI;
}

// 観測更新(センサの値と比較して重みを更新)
void ParticleFilter::measurement_update()
{

}

// 重みの正規化
void ParticleFilter::normalize_weight()
{
    double sum = 0.0;
    for(const auto &p : particles_)
        sum += p.get_weight();

    for(auto &p : particles_)
    {
        double new_weight = p.get_weight() / sum;
        p.set_weight(new_weight)
    }
}

// リサンプリング
void ParticleFilter::resampling()
{
    std::vector<Particle> new_particles;

    std::uniform_int_distribution<> int_dist(0,N_-1);
    std::uniform_real_distribution<> double_dist(0.0,get_max_weight()*2.0);

    int index = int_dist(engine);
    double beta = 0.0;

    for(int i=0; i<N_; i++)
    {
        beta += double_dist;
        while(beta > particles_[index].get_weight())
        {
            beta -= particles_[index].get_weight();
            index = (index+1) % N_;
        }
        new_particles.push_back(particles_[index]);
    }

    particles_ = new_particles;
    reset_weight();
}

// particles_ から最大の重みを取得
double ParticleFilter::get_max_weight()
{
    double max = 0.0;

    for(const auto &p : particles_)
    {
        if(max <= p.get_weight())
            max = p.get_weight();
    }

    return max;
}

// 重みのリセット
void ParticleFilter::reset_weight()
{
    for(auto &p : particles_)
        p.set_weight(1.0/particles_.size());
}

// ==== クラス Localizer ====
// コンストラクタ
Localizer::Localizer():private_nh("~")
{
    // パラメータの取得
    private_nh.getParam("hz",hz_);
    private_nh.getParam("N",N_);
    private_nh.getParam("INIT_X",INIT_X_);
    private_nh.getParam("INIT_Y",INIT_Y_);
    private_nh.getParam("INIT_YAW",INIT_YAW_);
    private_nh.getParam("INIT_X_COV",INIT_X_COV_);
    private_nh.getParam("INIT_Y_COV",INIT_Y_COV_);
    private_nh.getParam("INIT_YAW_COV",INIT_YAW_COV_);

    // Subscriber
    sub_laser = nh.subscribe("/scan", 10, &Localizer::laser_callback, this);
    sub_odometry = nh.subscribe("/roomba/odometry", 10, &Localizer::odometry_callback, this);
    sub_map_ = nh.subscribe("/map", 10, &Localizer::map_callback, this);

    // Publisher
    pub_estimated_pose_ = nh.advertise<geometry_msgs::PoseStamped>("/estimated_pose", 1);
    pub_particle_cloud_ = nh.advertise<geometry_msgs::PoseArray>("/particle_cloud", 1);
}

// 各種コールバック関数
void Localizer::odometry_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if(!get_odometry_)
    {
        prev_odometry_ = *msg;
        get_odometry_ = true;
    }
    else
    {
        prev_odometry_ = last_odometry_;
    }

    last_odometry_ = *msg;
}

void Localizer::map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    map_ = *msg;
}

void Localizer::laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    laser_ = *msg;
}

void Localizer::process()
{
    ros::Rate loop_rate(hz_);

    while(ros::ok())
    {
        if(init_request_)
        {
            pf_ -> initialize();
            init_request_ = false;
        }
        if(get_odometry_)
        {
            pf_ -> motion_update(last_odometry_, prev_odometry_);
        }
        pf -> measurement_update();
        pf -> resampling();

        publish_particles();

        ros::spinOnce();
        loop_rate.sleep();
    }
}

Localizer::publish_particles()
{
    particle_cloud_msg_.header.stamp = ros::Time::now();        // 現在時刻を取得
    particle_cloud_msg_.header.frame_id = "map";                // パブリッシュするフレームの設定
    particle_cloud_msg_.poses.resize(pf_ -> particles_.size()); // パブリッシュする PoseArray 配列のサイズを particles_ のサイズに合わせる

    for(int i=0; i< (pf_ -> particles_.size()); i++)
    {
        particle_cloud_msg_.poses[i].position.x = pf_ -> particles_[i].get_pose_x();
        particle_cloud_msg_.poses[i].position.y = pf_ -> particles_[i].get_pose_y();
        particle_cloud_msg_.poses[i].position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, pf_ -> particles_[i].get_pose_yaw());
        tf2::convert(q, particle_cloud_msg_.poses[i].orientation);
    }

    pub_particle_cloud_.publish(particle_cloud_msg_);
}

// ==== メイン関数 ====
int main(int argc, char **argv)
{
    ros::init(argc, argv, "localizer");
    Localizer localizer;
    localizer.process();

    return 0;
}
