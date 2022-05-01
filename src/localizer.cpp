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
    private_nh_.getParam("distance_noise_rate",distance_noise_rate_);
    private_nh_.getParam("rotation_noise_rate",rotation_noise_rate_);
    private_nh_.getParam("laser_noise_rate", laser_noise_rate_);
    private_nh_.getParam("laser_step", laser_step_);
    private_nh_.getParam("laser_ignore_range", laser_ignore_range_);
    private_nh_.getParam("expansion_limit", expansion_limit_);
    private_nh_.getParam("alpha_th", alpha_th_);
    private_nh_.getParam("alpha_slow_th", alpha_slow_th_);
    private_nh_.getParam("alpha_fast_th", alpha_fast_th_);
    private_nh_.getParam("expansion_rate", expansion_rate_);
    private_nh_.getParam("adaptive_reset_noise_rate", adaptive_reset_noise_rate_);

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
    if(!odometry_got_)
    {
        previous_odometry_ = *msg;
        odometry_got_ = true;
    }
    else
    {
        previous_odometry_ = current_odometry_;
    }

    current_odometry_ = *msg;
}

void Localizer::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    map_ = *msg;
    map_got_ = true;
}

void Localizer::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laser_ = *msg;
    laser_got_ = true;
}

// ノイズを乗せた値を返す関数
double Localizer::set_noise(double mu, double cov)
{
    std::normal_distribution<> dist(mu,cov);
    return dist(engine);
}

// 適切な角度 (-π ~ π) を返す関数
double Localizer::optimize_angle(double angle)
{
    if(angle > M_PI)
        angle -= 2*M_PI;
    if(angle < -M_PI)
        angle += 2*M_PI;

    return angle;
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

// 動作更新
void Localizer::motion_update()
{
    double dx = current_odometry_.pose.pose.position.x - previous_odometry_.pose.pose.position.x;
    double dy = current_odometry_.pose.pose.position.y - previous_odometry_.pose.pose.position.y;
    double curt_yaw = tf2::getYaw(current_odometry_.pose.pose.orientation);
    double prev_yaw = tf2::getYaw(previous_odometry_.pose.pose.orientation);
    double dyaw = optimize_angle(curt_yaw - prev_yaw);

    double distance = hypot(dx, dy);
    double direction = optimize_angle(atan2(dy,dx) - prev_yaw);

    for(auto& p : particles_)
        move(p, distance, direction, dyaw);
}

// パーティクルの移動
void Localizer::move(Particle& p, double distance, double direction, double rotation)
{
    distance += set_noise(0.0, distance * distance_noise_rate_);
    direction += set_noise(0.0, direction * rotation_noise_rate_);
    rotation += set_noise(0.0, rotation * rotation_noise_rate_);

    double x = p.get_pose_x() + distance * cos( optimize_angle(direction + p.get_pose_yaw()) );
    double y = p.get_pose_y() + distance * sin( optimize_angle(direction + p.get_pose_yaw()) );
    double yaw = optimize_angle(p.get_pose_yaw() + rotation);

    p.set_pose(x, y, yaw);
}

// 観測更新
void Localizer::measurement_update()
{
    for(auto &p : particles_)
    {
        double new_weight = calc_weight(p);
        p.set_weight(new_weight);
    }
    normalize_weight();
    estimate_pose();
    double normalized_alpha = alpha_ / (laser_.ranges.size() / laser_step_ * num_);

    if(normalized_alpha < alpha_th_ && expansion_count_ < expansion_limit_)
    {
        expansion_count_ ++;
        expansion_reset();
    }
    else
    {
        resampling();
        expansion_count_ = 0;
    }

}

double Localizer::calc_weight(Particle& p)
{
    double weight = 0.0;

    double angle = optimize_angle(p.get_pose_yaw() + laser_.angle_min);
    double angle_step = laser_.angle_increment;
    int limit = laser_.ranges.size();

    for(int i=0; i < limit; i += laser_step_)
    {
        if(laser_.ranges[i] > laser_ignore_range_)
        {
            double sigma = laser_.ranges[i] * laser_noise_rate_;

            double laser_dist = set_noise(laser_.ranges[i], sigma);
            double map_dist = dist_on_map(p.get_pose_x(), p.get_pose_y(), laser_dist, angle);

            weight += likelihood(map_dist, laser_dist, sigma);
        }

        angle = optimize_angle(angle + angle_step * laser_step_);
    }
    return weight;
}

double Localizer::likelihood(double x, double mu, double sigma)
{
    double ans = exp( - std::pow(x - mu, 2) / (2.0 * std::pow(sigma, 2)) ) / ( sqrt( 2.0 * M_PI ) * sigma );

    return ans;
}

double Localizer::dist_on_map(double map_x, double map_y, double laser_dist, double laser_angle)
{
    double distance = 0.0;

    double search_step = map_.info.resolution;
    double search_limit = laser_.range_max;

    for(distance; distance <= search_limit; distance += search_step)
    {
        map_x += search_step * cos(laser_angle);
        map_y += search_step * sin(laser_angle);

        int map_occupancy = get_map_occupancy(map_x, map_y);

        if(map_occupancy != 0)    return distance;
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
    alpha_ = 0.0;
    for(const auto &p : particles_)
        alpha_ += p.get_weight();

    for(auto &p : particles_)
    {
        double new_weight = p.get_weight() / alpha_;
        p.set_weight(new_weight);
    }

}

void Localizer::expansion_reset()
{
    std::vector<Particle> new_particles;
    new_particles = particles_;

    for(int i=0; i<num_; i++)
    {
        double x = set_noise(new_particles[i].get_pose_x(), expansion_rate_);
        double y = set_noise(new_particles[i].get_pose_y(), expansion_rate_);
        double yaw = set_noise(new_particles[i].get_pose_yaw(), expansion_rate_);
        new_particles[i].set_pose(x, y, yaw);
    }

    particles_ = new_particles;
    reset_weight();
}

// リサンプリング
void Localizer::resampling()
{
    std::vector<Particle> new_particles;
    new_particles.reserve(num_);

    std::uniform_int_distribution<> int_dist(0,num_-1);
    std::uniform_real_distribution<> double_dist(0.0,get_max_weight()*2.0);

    alpha_slow_ += alpha_slow_th_ * (alpha_ - alpha_slow_);
    alpha_fast_ += alpha_fast_th_ * (alpha_ - alpha_fast_);

    num_replace_ = (int) (num_ * std::max( 0.0, 1.0 - alpha_fast_ / alpha_slow_));

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
        if(new_particles.size() < num_replace_)
        {
            double x = set_noise(estimated_pose_.get_pose_x(), adaptive_reset_noise_rate_);
            double y = set_noise(estimated_pose_.get_pose_y(), adaptive_reset_noise_rate_);
            double yaw = set_noise(estimated_pose_.get_pose_yaw(), adaptive_reset_noise_rate_);
            Particle p(x, y, yaw);
            new_particles.push_back(p);
        }
        else
            new_particles.push_back(particles_[index]);
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
        if(max <= p.get_weight())   max = p.get_weight();
    }

    return max;
}

// particles_ の重みをリセット
void Localizer::reset_weight()
{
    for(auto &p : particles_)
        p.set_weight(1.0/particles_.size());
}

void Localizer::estimate_pose()
{
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double max_weight = get_max_weight();

    for(const auto &p : particles_)
    {
        x += p.get_pose_x() * p.get_weight();
        y += p.get_pose_y() * p.get_weight();
        if(p.get_weight() == max_weight)
            yaw = p.get_pose_yaw();
    }

    estimated_pose_.set_pose(x, y, yaw);
}

void Localizer::process()
{
    ros::Rate loop_rate(hz_);
    bool start = false;

    while(ros::ok())
    {
        if(map_got_ && odometry_got_ && laser_got_)
        {
            if(init_request_)
            {
                initialize();
                init_request_ = false;
            }
            if(current_odometry_ != previous_odometry_)
                start = true;
            if(start)
            {
                motion_update();
                measurement_update();
            }

            publish_particles();
            publish_estimated_pose();
            try
            {
                broadcast_roomba_state();
            }
            catch(tf::TransformException &ex)
            {
                ROS_ERROR("%s", ex.what());
            }

        }
        ros::spinOnce();
        loop_rate.sleep();

    }
}

void Localizer::publish_particles()
{
    particle_cloud_msg_.header.stamp = ros::Time::now(); // 現在時刻を取得
    particle_cloud_msg_.header.frame_id = "map"; // パブリッシュするフレームの設定
    particle_cloud_msg_.poses.resize(particles_.size()); // パブリッシュする PoseArray 配列のサイズを particles_ のサイズに合わせる

    for(int i=0; i < particles_.size(); i++)
    {
        particle_cloud_msg_.poses[i].position.x = particles_[i].get_pose_x();
        particle_cloud_msg_.poses[i].position.y = particles_[i].get_pose_y();
        particle_cloud_msg_.poses[i].position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, particles_[i].get_pose_yaw());
        tf2::convert(q, particle_cloud_msg_.poses[i].orientation);
    }

    pub_particle_cloud_.publish(particle_cloud_msg_);
}

void Localizer::publish_estimated_pose()
{
    estimated_pose_msg_.header.stamp = ros::Time::now();
    estimated_pose_msg_.header.frame_id = "map";

    estimated_pose_msg_.pose.position.x = estimated_pose_.get_pose_x();
    estimated_pose_msg_.pose.position.y = estimated_pose_.get_pose_y();

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, estimated_pose_.get_pose_yaw());
    tf2::convert(q, estimated_pose_msg_.pose.orientation);

    pub_estimated_pose_.publish(estimated_pose_msg_);
}

void Localizer::broadcast_roomba_state()
{
    double map_to_base_x = estimated_pose_.get_pose_x();
    double map_to_base_y = estimated_pose_.get_pose_y();
    double map_to_base_yaw = estimated_pose_.get_pose_yaw();

    double odom_to_base_x = current_odometry_.pose.pose.position.x;
    double odom_to_base_y = current_odometry_.pose.pose.position.y;
    double odom_to_base_yaw = tf2::getYaw(current_odometry_.pose.pose.orientation);

    double roomba_state_yaw = optimize_angle(map_to_base_yaw - odom_to_base_yaw);
    double roomba_state_x = map_to_base_x - odom_to_base_x * cos(roomba_state_yaw) + odom_to_base_y * sin(roomba_state_yaw);
    double roomba_state_y = map_to_base_y - odom_to_base_x * sin(roomba_state_yaw) - odom_to_base_y * cos(roomba_state_yaw);
    geometry_msgs::Quaternion roomba_state_q;
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(roomba_state_yaw), roomba_state_q);

    geometry_msgs::TransformStamped roomba_state;
    roomba_state.header.stamp = ros::Time::now();

    roomba_state.header.frame_id = "map";
    roomba_state.child_frame_id = "odom";

    roomba_state.transform.translation.x = roomba_state_x;
    roomba_state.transform.translation.y = roomba_state_y;
    roomba_state.transform.translation.z = 0.0;
    roomba_state.transform.rotation = roomba_state_q;

    roomba_state_broadcaster_.sendTransform(roomba_state);
}

// ==== メイン関数 ====
int main(int argc, char **argv)
{
    ros::init(argc, argv, "localizer");
    Localizer localizer;
    localizer.process();

    return 0;
}
