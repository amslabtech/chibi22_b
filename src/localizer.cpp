#include "localizer/localizer.h"

// 疑似乱数生成器
std::random_device seed_gen;
std::mt19937 engine(seed_gen());

// ==== クラス Particle ====
// コンストラクタ
Localizer::Particle::Particle(double x, double y, double yaw, double weight)
{
    set_pose(x,y,yaw);
    set_weight(weight);
}

// pose の設定
void Localizer::Particle::set_pose(double x, double y, double yaw)
{
    x_ = x;
    y_ = y;
    yaw_ = yaw;
}

// 重みの設定
void Localizer::Particle::set_weight(double weight)
{
    weight_ = weight;
}



// ==== クラス Localizer ====
// コンストラクタ
Localizer::Localizer():private_nh_("~")
{
    // パラメータの取得
    private_nh_.getParam("hz",hz_);
    private_nh_.getParam("num",num_);
    private_nh_.getParam("init_x",init_x_);
    private_nh_.getParam("init_y",init_y_);
    private_nh_.getParam("init_yaw",init_yaw_);
    private_nh_.getParam("x_cov",x_cov_);
    private_nh_.getParam("y_cov",y_cov_);
    private_nh_.getParam("yaw_cov",yaw_cov_);
    private_nh_.getParam("distance_noise_ratio",distance_noise_ratio_);
    private_nh_.getParam("rotation_noise_ratio",rotation_noise_ratio_);
    private_nh_.getParam("laser_noise_ratio", laser_noise_ratio_);

    // Subscriber
    sub_laser_ = nh_.subscribe("/scan", 10, &Localizer::laser_callback, this);
    sub_odometry_ = nh_.subscribe("/roomba/odometry", 10, &Localizer::odometry_callback, this);
    sub_map_ = nh_.subscribe("/map", 10, &Localizer::map_callback, this);

    // Publisher
    pub_estimated_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/estimated_pose", 1);
    pub_particle_cloud_ = nh_.advertise<geometry_msgs::PoseArray>("/particle_cloud", 1);

}

// 各種コールバック関数
void Localizer::odometry_callback(const nav_msgs::Odometry::ConstPtr& msg)
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

void Localizer::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    map_ = *msg;
}

void Localizer::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laser_ = *msg;
}

// 初期化処理
void Localizer::initialize()
{
    double w = 1.0/(double)num_;

    for(int i=0; i<num_; i++){
        double x = set_noise(init_x_,x_cov_);
        double y = set_noise(init_y_,y_cov_);
        double yaw = set_noise(init_yaw_,yaw_cov_);
        Particle p(x, y, yaw, w);
        particles_.push_back(p);
    }
}

// 移動量の算出
void Localizer::motion_update(const nav_msgs::Odometry last, const nav_msgs::Odometry prev)
{
    double dx = last.pose.pose.position.x - prev.pose.pose.position.x;
    double dy = last.pose.pose.position.y - prev.pose.pose.position.y;
    double last_yaw = tf2::getYaw(last.pose.pose.orientation);
    double prev_yaw = tf2::getYaw(prev.pose.pose.orientation);
    double dyaw = optimize_angle(last_yaw - prev_yaw);

    double distance = hypot(dx, dy);
    double direction = optimize_angle(atan2(dy,dx) - prev_yaw);

    for(auto& p : particles_)
        move(p, distance, direction, dyaw);
}

// パーティクルの移動
void Localizer::move(Particle& p, double distance, double direction, double rotation)
{
    distance += set_noise(0.0, distance * distance_noise_ratio_);
    direction += set_noise(0.0, direction * rotation_noise_ratio_);
    rotation += set_noise(0.0, rotation * rotation_noise_ratio_);

    double new_x = p.get_pose_x() + distance * cos( optimize_angle(direction + p.get_pose_yaw()) );
    double new_y = p.get_pose_y() + distance * sin( optimize_angle(direction + p.get_pose_yaw()) );
    double new_yaw = optimize_angle(p.get_pose_yaw() + rotation);

    p.set_pose(new_x, new_y, new_yaw);
}

// ノイズを乗せた値を返す関数
double Localizer::set_noise(double mu, double cov)
{
    std::normal_distribution<> dist(mu,cov);
    return dist(engine);
}

// 適切な角度 (-π ~ π) を返す
double Localizer::optimize_angle(double angle)
{
    if(angle > M_PI)
        angle -= 2*M_PI;
    if(angle < -M_PI)
        angle += 2*M_PI;

    return angle;
}

// 観測更新(センサの値と比較して重みを更新)
void Localizer::measurement_update()
{
    for(auto& p : particles_)
    {
        double new_weight = calc_weight(p);
        p.set_weight(new_weight);
    }

    normalize_weight();

    resampling();
}

double Localizer::calc_weight(Particle& p)
{
    double weight = 0.0;

    double angle = optimize_angle(p.get_pose_yaw() + laser_.angle_min);
    double angle_step = laser_.angle_increment;
    int limit = laser_.ranges.size();

    for(int index=0; index < limit; index++)
    {
        double sigma = laser_.ranges[index] * laser_noise_ratio_;

        double laser_dist = set_noise(laser_.ranges[index], sigma);
        double map_dist = dist_on_map(p.get_pose_x(), p.get_pose_y(), laser_dist, angle);

        weight += likelihood(map_dist, laser_dist, sigma);

        angle = optimize_angle(angle + angle_step);
    }

    return weight;
}

double Localizer::likelihood(double x, double mu, double sigma)
{
    double ans = exp( - std::pow(x - mu, 2) / 2 * std::pow(sigma, 2) ) / sqrt( 2 * M_PI * std::pow(sigma, 2) );

    return ans;
}

double Localizer::dist_on_map(double map_x, double map_y, const double laser_dist, const double laser_angle)
{
    double distance = 0.0;
    double search_step = map_.info.resolution;
    double search_limit = std::min(laser_dist,(double)laser_.range_max);

    for(distance; distance <= search_limit; distance += search_step)
    {
        map_x += distance * cos(laser_angle);
        map_y += distance * sin(laser_angle);

        int map_occupancy = get_map_occupancy(map_x, map_y);

        if(map_occupancy == 100)
            return distance;
        if(map_occupancy == -1)
            return search_limit * 2.0;
    }
    return search_limit;
}

int Localizer::get_map_occupancy(double x, double y)
{
    double origin_x = map_.info.origin.position.x;
    double origin_y = map_.info.origin.position.y;
    double resolution = map_.info.resolution;
    double width = map_.info.width;

    int mx = (int)floor( (x - origin_x) / resolution );
    int my = (int)floor( (y - origin_y) / resolution );

    int occupancy = map_.data[mx + my*width];

    return occupancy;
}

// 重みの正規化
void Localizer::normalize_weight()
{
    double sum = 0.0;
    for(const auto &p : particles_)
        sum += p.get_weight();

    for(auto &p : particles_)
    {
        double new_weight = p.get_weight() / sum;
        p.set_weight(new_weight);
    }
}

// リサンプリング
void Localizer::resampling()
{
    std::vector<Particle> new_particles;

    std::uniform_int_distribution<> int_dist(0,num_-1);
    std::uniform_real_distribution<> double_dist(0.0,get_max_weight()*2.0);

    int index = int_dist(engine);
    double beta = 0.0;

    for(int i=0; i<num_; i++)
    {
        beta += double_dist(engine);
        while(beta > particles_[index].get_weight())
        {
            beta -= particles_[index].get_weight();
            index = (index+1) % num_;
        }
        new_particles.push_back(particles_[index]);

        //ROS_INFO_STREAM("chosen : "<<"x="<<particles_[index].get_pose_x()<<"y="<<particles_[index].get_pose_y()<<"yaw="<<particles_[index].get_pose_yaw()<<"weight="<<particles_[index].get_weight());
    }

    particles_ = new_particles;
    reset_weight();
}

// particles_ から最大の重みを取得
double Localizer::get_max_weight()
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
void Localizer::reset_weight()
{
    for(auto &p : particles_)
        p.set_weight(1.0/particles_.size());
}

void Localizer::process()
{
    ros::Rate loop_rate(hz_);

    while(ros::ok())
    {
        if(init_request_)
        {
            initialize();
            init_request_ = false;
        }
        if(get_odometry_)
        {
            motion_update(last_odometry_, prev_odometry_);
        }
        //measurement_update();

        publish_particles();

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void Localizer::publish_particles()
{
    particle_cloud_msg_.header.stamp = ros::Time::now(); // 現在時刻を取得
    particle_cloud_msg_.header.frame_id = "map"; // パブリッシュするフレームの設定
    particle_cloud_msg_.poses.resize(particles_.size()); // パブリッシュする PoseArray 配列のサイズを particles_ のサイズに合わせる

    for(int i=0; i< (particles_.size()); i++)
    {
        particle_cloud_msg_.poses[i].position.x = particles_[i].get_pose_x();
        particle_cloud_msg_.poses[i].position.y = particles_[i].get_pose_y();
        particle_cloud_msg_.poses[i].position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, particles_[i].get_pose_yaw());
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
