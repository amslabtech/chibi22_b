#include "global_path_planner/global_path_planner.h"

AStar::AStar():private_nh_("~")
{
    private_nh_.param("hz", hz_, {10});
    private_nh_.param("map_checker", map_checker_, {false});
    private_nh_.param("path_checker", path_checker_, {false});
    private_nh_.param("is_reached", is_reached_, {false});
    private_nh_.param("wall_cost", wall_cost_, {1e10});

    sub_map_ = nh_.subscribe("/new_map", 10, &AStar::map_callback, this);                   //mapデータの受信
    pub_path_ = nh_.advertise<nav_msgs::Path>("/global_path", 1);                           //出力するパス
    pub_wp_path_ = nh_.advertise<nav_msgs::Path>("/wp_path", 1);                            //1辺
}

//mapを格納
void AStar::map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    map_data_ = *msg;
    if(map_data_.data.size() == 0)
    {
        std::cout << "Error in set_map" << std::endl;
        sleep(3);
        return;
    }
    else
    {
        row_ = map_data_.info.height;                                                       //マップの大きさを保存
        col_ = map_data_.info.width;
        resolution_ = map_data_.info.resolution;

        map_grid_ = std::vector<std::vector<int>>(row_, std::vector<int>(col_,0));          //map_gridの初期化

        //マップデータを変換
        for(int i=0;i<row_;i++)
        {
            for(int j=0;j<col_;j++)
            {
                map_grid_[i][j] = map_data_.data[i+j*row_];
            }
        }

        origin_.x = map_data_.info.origin.position.x;                                      //原点の保存
        origin_.y = map_data_.info.origin.position.y;

        global_path_.header = map_data_.header;
        wp_path_.header = map_data_.header;
        path_point_.header = map_data_.header;

        map_checker_ = true;

        ref_time_ = time(NULL);

        std::cout << "row:" << row_ << " col_:" << col_ << std::endl;
        std::cout << "set_map finished\n" << std::endl;
        sleep(1);
    }
}

//ウェイポイントの座標を設定
void AStar::set_waypoint()
{
    std::cout << "origin_.x:" << origin_.x << " y:" << origin_.y << std::endl;
    map_origin_x_ = origin_.x / resolution_;
    map_origin_y_ = origin_.y / resolution_;

    int x0,y0,x1,y1,x2;
    x0 =  0;
    y0 =  0;
    x1 =  11.5/resolution_;
    y1 =  14.4/resolution_;
    x2 = -22.0/resolution_;

    waypoint_ = {
        {x0,y0},
        {x1,y0},
        {x1,y1},
        {x2,y1},
        {x2,y0},
        {x0,y0},
    };
}

//ノードを格納するリストの初期化
void AStar::set_NodeList(std::vector<Node> &list)
{
    int ListSize = list.size();
    for(int i=0; i<ListSize; i++)
    {
        list.pop_back();
    }
}

//ノードリストの表示
void AStar::show_NodeList(std::vector<Node> list)
{
    for(int i=0; i<list.size(); i++)
    {
        std::cout << "i:" << i << " x:" << list[i].x << " y:" << list[i].y << " g:" << list[i].g << " f:" << list[i].f << std::endl;
    }
}

//heuristic_関数の作成
void AStar::make_heuristic(int phase_){
    heuristic_ = std::vector<std::vector<int>>(row_, std::vector<int>(col_,0));                //heuristic_関数の初期化
    for(int i=0; i<row_; i++)
    {
        for(int j=0; j<col_; j++)
        {
            heuristic_[i][j] = abs(waypoint_[phase_][0] + abs(map_origin_x_) - i) + abs(waypoint_[phase_][1] + abs(map_origin_y_) - j);    //ゴールからの距離で設定
        }
    }
    std::cout << "test h-v start:" << heuristic_[waypoint_[phase_-1][0] + abs(map_origin_x_)][waypoint_[phase_-1][0] + abs(map_origin_y_)] << " goal:" << heuristic_[waypoint_[phase_][0] + abs(map_origin_x_)][waypoint_[phase_][1] + abs(map_origin_y_)] << std::endl;
    std::cout << "made heuristic" << std::endl;
}

//コストは最小か
bool AStar::is_low_cost(int i){
    if(open_list_[i].f < min_f_ && open_list_[i].f >= 0)
        return true;
    else
        return false;
}

//既存のパスと接しているか
bool AStar::is_contact(int i){
    if(fabs(open_list_[i].x-pre_p_Node_.x) == 0 && fabs(open_list_[i].y-pre_p_Node_.y) <= 1)
        return true;
    else if(fabs(open_list_[i].x-pre_p_Node_.x) <= 1 && fabs(open_list_[i].y-pre_p_Node_.y) == 0)
        return true;
    else
        return false;
}

//進行方向の記録
int AStar::set_current_delta(int i){
    if(open_list_[i].x - pre_p_Node_.x == 0 && open_list_[i].y - pre_p_Node_.y == 1)
        return 0;
    else if(open_list_[i].x - pre_p_Node_.x == 1 && open_list_[i].y - pre_p_Node_.y == 0)
        return 1;
    else if(open_list_[i].x - pre_p_Node_.x == 0 && open_list_[i].y - pre_p_Node_.y == -1)
        return 2;
    else if(open_list_[i].x - pre_p_Node_.x == -1 && open_list_[i].y - pre_p_Node_.y == 0)
        return 3;
    else
        return 5;
}

void AStar::update_p_node(int i)
{
    min_f_ = open_list_[i].f;                                       //最小コストの更新
    pre_dist_wp_ = dist_wp_;                                        //ウェイポイントとの距離の記録
    p_Node_ = open_list_[i];                                        //親ノードに設定
    open_list_[i].f = 1e10;                                         //Openノードを選択できなくする
}

bool AStar::is_contacted_wall()
{
    int x,y;
    x = pre_p_Node_.x - map_origin_x_;
    y = pre_p_Node_.x - map_origin_y_;
    if(map_grid_[x+1][y+1]!=0 || map_grid_[x+1][y-1]!=0 || map_grid_[x-1][y+1]!=0 || map_grid_[x-1][y-1]!=0)
        return true;
    else
        return false;
}

bool AStar::is_neared()
{
    if(dist_wp_ <= pre_dist_wp_)
        return true;
    else
        return false;
}

//親ノードの探索
void AStar::set_pNode()
{
    pre_p_Node_ = p_Node_;  //直前の親ノードの保存
    p_Node_ = init_Node_;   //親ノードの初期化
    p_Node_.g = 1;
    p_Node_.f = 1e10;
    min_f_ = 1e10;          //各変数の初期化
    dist_wp_ = 1e20;
    pre_dist_wp_ = 1e20;
    current_delta_ = 5;

    //最小探索
    int count = 0;
    for(int i=open_list_.size()-1; i>=0; i--)                                                   //すべての子ノードを探索
    {
        dist_wp_ = std::sqrt(std::pow(open_list_[i].x - x_waypoint_, 2) + std::pow(open_list_[i].y - y_waypoint_, 2));
        if(is_low_cost(i) && is_contact(i) && is_neared())
        {
            if(x_waypoint_ != pre_p_Node_.x && y_waypoint_ != pre_p_Node_.y)                    //斜め移動するとき
            {
                current_delta_ = set_current_delta(i);                                          //進行方向の記録
                if(is_contacted_wall())                                                         //壁に接しているとき
                {
                    if(fabs(x_waypoint_-open_list_[i].x) > fabs(y_waypoint_-open_list_[i].y))   //進行方向による場合分け
                    {
                        if(open_list_[i].x == pre_p_Node_.x)
                            update_p_node(i);
                        else
                            std::cout << "not x" << std::endl;
                    }
                    else
                    {
                        if(open_list_[i].y == pre_p_Node_.y)
                            update_p_node(i);
                        else
                            std::cout << "not y" << std::endl;
                    }
                }
                else if(current_delta_ != pre_delta_)
                    update_p_node(i);
                else
                    std::cout << "error in delta_" << std::endl;
            }
            else
                update_p_node(i);
        }
    }
    if(current_delta_ != 5)
        pre_delta_ = current_delta_;

    //エラー処理
    if(min_f_ >= 1e10)
    {
        std::cout << "Error in set_pNode\n" << std::endl;
        sleep(3);
        exit(0);
    }
}

//子ノードの設定
void AStar::set_kNode()
{
    //子ノードの初期化
    set_NodeList(k_Node_);
    for(int i=0; i<delta_.size(); i++)
    {
        k_Node_.push_back(init_Node_);
    }
    //子ノードの設定
    for(int i=0; i<delta_.size(); i++)
    {
        k_Node_[i].x = p_Node_.x + delta_[i][0];                                        //子ノードの設定
        k_Node_[i].y = p_Node_.y + delta_[i][1];
                //子ノードのコスト求める
        if(map_grid_[k_Node_[i].x-map_origin_x_][k_Node_[i].y-map_origin_y_] == 0)      //障害物が存在したらg値大
            k_Node_[i].g = p_Node_.g + 1;
        else
            k_Node_[i].g = wall_cost_;
        k_Node_[i].f = heuristic_[k_Node_[i].x-map_origin_x_][k_Node_[i].y-map_origin_y_] + k_Node_[i].g;
    }
    close_list_.push_back(p_Node_);                                                     //親ノードをCloseリストに移動
}

//Openリストに入っているか調べる
int AStar::is_open(int num)
{
    int i=0;
    for(i=open_list_.size()-1; i>=0; i--)
    {
        if((k_Node_[num].x == open_list_[i].x) && (k_Node_[num].y == open_list_[i].y))
            return i;
    }
    return 0;
}

//Closeリストに入っているか調べる
int AStar::is_close(int num)
{
    int i=0;
    for(i=close_list_.size()-1; i>=0; i--)
    {
        if((k_Node_[num].x == close_list_[i].x) && (k_Node_[num].y == close_list_[i].y))
            return i;
    }
    return 0;
}

//親ノードをパスに追加
void AStar::add_path()
{
    geometry_msgs::PoseStamped path_point_;
    path_point_.pose.position.x = double(p_Node_.x * resolution_);
    path_point_.pose.position.y = double(p_Node_.y * resolution_);
    path_point_.pose.orientation.w = 1;
    //test
    // wp_path_.poses.push_back(path_point_);
    // std::cout << "test add x:" << p_Node_.x << " y:" << p_Node_.y << std::endl;
}

//ノードのコストを更新し、親ノードを記録する
void AStar::update_cost()
{
    is_updated_ = false;
    for(int i=0; i<k_Node_.size(); i++)                                 //すべての子ノードについて
    {
        for(int j=0; j<open_list_.size() && j<close_list_.size(); j++)
        {
            if(!is_close(i)==0)                                         //Closeリストに入っているか調べる
            {
                if(k_Node_[i].f < close_list_[is_close(i)].f)           //コストが小さければ更新
                {
                    close_list_[j].f = k_Node_[i].f;
                    is_updated_ = true;
                }
            }
            else if(!is_open(i)==0)                                     //Openリストに入っているか調べる
            {
                if(k_Node_[i].f < open_list_[is_open(i)].f)             //コストが小さければ更新
                {
                    open_list_[j].f = k_Node_[i].f;
                    is_updated_ = true;
                }
            }
            else
            {
                open_list_.push_back(k_Node_[i]);                       //リストになければ追加
                is_updated_ = true;
            }
        }
    }
    if(is_updated_) add_path();                                         //親ノードを保存
}

//作成したパスをglobal_path_に追加
void AStar::make_path()
{
    add_path();
    global_path_.poses.insert(global_path_.poses.end(), wp_path_.poses.begin(), wp_path_.poses.end());
    pub_wp_path_.publish(wp_path_);                                     //確認用
}

void AStar::show_time()
{
    cur_time_ = time(NULL);
    std::cout << "実行時間：" << cur_time_ - ref_time_ << "s" << std::endl;
}

//A*を実行
void AStar::astar_process()
{
    std::cout << "-----astar_process will begin-----\n" << std::endl;

    set_waypoint();                                                     //ウェイポイントを作成
    std::cout << "test wp.size:" << waypoint_.size() << std::endl;

    //それぞれのウェイポイント間の経路を作成
    for(int phase_ = 1; phase_ < waypoint_.size(); phase_++)
    {
        is_reached_ = false;                                            //is_reachedの初期化
        make_heuristic(phase_);                                         //heuristic_関数の作成
        set_NodeList(open_list_);                                       //openリストcloseリストの初期化
        set_NodeList(close_list_);
        wp_path_.poses.clear();                                         //ウェイポイント間のパスの初期化
        x_waypoint_ = waypoint_[phase_][0];                             //ウェイポイントの保存
        y_waypoint_ = waypoint_[phase_][1];
        std::cout << "test wp.x:" << x_waypoint_ << " y:" <<  y_waypoint_ << std::endl;

        //スタートノードを親ノードに
        origin_.x = waypoint_[phase_-1][0];
        origin_.y = waypoint_[phase_-1][1];
        origin_.g = 0;
        origin_.f = heuristic_[origin_.x-map_origin_x_][origin_.y-map_origin_y_] + origin_.g;
        open_list_.push_back(origin_);
        p_Node_ = origin_;
        std::cout << "test origin.x:" << origin_.x << " y:" << origin_.y << " f:" << origin_.f << std::endl;

        while(!is_reached_)                                             //ウェイポイントに到達するまで
        {
            set_pNode();                                                //親ノードを設定
            if(p_Node_.x == x_waypoint_ && p_Node_.y == y_waypoint_)    //親ノードとゴールノードが一致したらis_reachedをTrueに
            {
                is_reached_ = true;
                std::cout << "reached waypoint" << std::endl;
            }
            else if(map_grid_[p_Node_.x-map_origin_x_][p_Node_.y-map_origin_y_] != 0)
            {
                ROS_ERROR_STREAM("NULL NODE");
                exit(0);
            }
            else
            {
                set_kNode();                                            //子ノードを設定
                update_cost();                                          //コストを更新し、ノードをパスに追加
            }
        }
        make_path();                                                    //ウェイポイント間の経路をつなぐ
        std::cout << "test fin phase:" << phase_ << std::endl;
        std::cout << std::endl;
    }
}//A*process

void AStar::process()
{
    std::cout << "\n-----GlobalPathPlanner will begin-----\n" << std::endl;
    ros::Rate loop_rate(hz_);
    while(ros::ok())
    {
        if(map_checker_ && !path_checker_)
        {
            astar_process();
            pub_path_.publish(global_path_);
            show_time();
            path_checker_ = true;
        }
        if(path_checker_)
            exit(0);
        ros::spinOnce();
        loop_rate.sleep();
    }
    std::cout << "test something error" << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "global_path_planner");
    AStar astar;
    astar.process();
    return 0;
}

