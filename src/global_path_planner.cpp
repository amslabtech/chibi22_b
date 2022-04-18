#include "global_path_planner/global_path_planner.h"

//Todo
//角度の情報
//処理の軽量化

AStar::AStar():private_nh("~")
{
    private_nh.param("hz", hz, {10});
    private_nh.param("map_checker", map_checker, {false});
    private_nh.param("path_checker", path_checker, {false});
    private_nh.param("is_reached", is_reached, {false});
    private_nh.param("wall_cost", wall_cost, {1e10});

    sub_map = nh.subscribe("map", 10, &AStar::map_callback, this);                      //mapデータの受信
    pub_path = nh.advertise<nav_msgs::Path>("global_path", 1);                          //出力するパス
    pub_wp_path = nh.advertise<nav_msgs::Path>("wp_path", 1);                           //1辺
    pub_wp = nh.advertise<geometry_msgs::PointStamped>("open_set_rviz", 1);             //rviz用
    pub_wall = nh.advertise<nav_msgs::OccupancyGrid>("wall_map", 1);
}



//mapを格納
void AStar::map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    map_data = *msg;
    if(map_data.data.size() == 0)
    {
        std::cout << "Error in set_map" << std::endl;
        sleep(3);
        return;
    }
    else
    {
        row = map_data.info.height;                                                     //マップの大きさを保存
        col = map_data.info.width;
        resolution = map_data.info.resolution;
        map_grid = std::vector<std::vector<int>>(row, std::vector<int>(col,0));         //map_gridの初期化
        occu_map_grid = std::vector<std::vector<int>>(row, std::vector<int>(col,0));

        //マップデータを変換
        for(int i=0;i<row;i++)
        {
            for(int j=0;j<col;j++)
            {
                map_grid[i][j] = map_data.data[i+j*row];
                if(i==2380 && 2595<j && j<2605)
                    map_grid[i][j] = 100;
            }
        }

        origin.x = map_data.info.origin.position.x;                                      //原点の保存
        origin.y = map_data.info.origin.position.y;

        // global_path.header.frame_id = "map";
        // wp_path.header.frame_id = "map";
        // way_point.header.frame_id = "map";
        // path_point.header.frame_id = "map";
        global_path.header = map_data.header;
        wp_path.header = map_data.header;
        way_point.header = map_data.header;
        path_point.header = map_data.header;

        map_checker = true;
        std::cout << "row:" << row << " col:" << col << std::endl;
        std::cout << "set_map finished\n" << std::endl;
        sleep(1);
    }
}

//ウェイポイントの座標を設定
void AStar::set_waypoint()
{
    int x1,x2,y1,y2;
    x1=1760;
    y1=2325;
    x2=2390;
    y2=2600;

    waypoint = {
        {1950,2520},
        {2595,2520},
        {2595,2820},
        {1950,2810},
        {1950,2520},
    };

    waypoint = {
        {x1,y1},
        {x2,y1},
        {x2,y2},
        {x1,y2},
        {x1,y1},
    };

    // waypoint = {
    //     {1760,2200},
    //     {1760,2700},
    //     {1770,2700},
    //     {1770,2200},
    //     {1780,2200},
    //     {1780,2700},
    //     {1790,2700},
    //     {1790,2200},
    //     {1800,2200},
    //     {1800,2700},
    //     {1810,2700},
    //     {1810,2200},
    //     {1820,2200},
    //     {1820,2700},
    //     {1830,2700},
    //     {1830,2200},
    //     {1840,2200},
    //     {1840,2700},
    //     {1850,2700},
    //     {1850,2200},
    //     {1860,2200},
    //     {1860,2700},
    //     {1870,2700},
    //     {1870,2200},
    // };

    // waypoint = {
    //     {2200,2100},
    //     {2800,2100},
    //     {2800,2150},
    //     {2200,2150},
    //     {2200,2200},
    //     {2800,2200},
    //     {2800,2250},
    //     {2200,2250},
    //     {2200,2300},
    //     {2800,2300},
    //     {2800,2350},
    //     {2200,2350},
    //     {2200,2400},
    //     {2800,2400},
    //     {2800,2450},
    //     {2200,2450},
    //     {2200,2500},
    //     {2800,2500},
    //     {2800,2520},
    //     {2200,2520},
    //     {2200,2540},
    //     {2800,2540},
    //     {2800,2560},
    //     {2200,2560},
    //     {2200,2580},
    //     {2800,2580},
    //     {2800,2600},
    //     {2200,2600},
    //     {2200,2650},
    //     {2800,2650},
    // };
}

//ノードを格納するリストの初期化
void AStar::set_NodeList(std::vector<Node> &list)
{
    int ListSize = list.size();
    for(int i=0; i<ListSize; i++)
    {
        list.pop_back();
    }
    std::cout << "set node list(size:" << list.size() << std::endl;
}

void AStar::show_NodeList(std::vector<Node> list)
{
    for(int i=0; i<list.size(); i++)
    {
        std::cout << "i:" << i << " x:" << list[i].x << " y:" << list[i].y << " g:" << list[i].g << " f:" << list[i].f << std::endl;
    }
}

//heuristic関数の作成
void AStar::make_heuristic(int Phase){
    heuristic = std::vector<std::vector<int>>(row, std::vector<int>(col,0));                //heuristic関数の初期化
    for(int i=0; i<row; i++)
    {
        for(int j=0; j<col; j++)
        {
            heuristic[i][j] = abs(waypoint[Phase][0] - i) + abs(waypoint[Phase][1] - j);    //ゴールからの距離で設定
        }
    }
    std::cout << "test h-v start:" << heuristic[waypoint[Phase-1][0]][waypoint[Phase-1][0]] << " goal:" << heuristic[waypoint[Phase][0]][waypoint[Phase][1]] << std::endl;
    std::cout << "made heuristic\n" << std::endl;
}

bool AStar::is_low_cost(int i){
    // std::cout << "test f:" << open_list[i].f << ", " << min_f << std::endl;
    if(open_list[i].f < min_f && open_list[i].f >= 0)
        return true;
    else
        return false;
}

bool AStar::is_contact(int i){
        if(fabs(open_list[i].x-pre_p_Node.x) == 0 && fabs(open_list[i].y-pre_p_Node.y) <= 1)
            return true;
        if(fabs(open_list[i].x-pre_p_Node.x) <= 1 && fabs(open_list[i].y-pre_p_Node.y) == 0)
            return true;
        else
            return false;
}

//親ノードの探索
void AStar::set_pNode()
{
    pre_p_Node = p_Node;                        //直前の親ノードの保存
    p_Node = init_Node;                         //親ノードの初期化
    p_Node.g = 1;
    p_Node.f = 1e10;
    min_f = 1e10;                              //各変数の初期化
    dist_wp = 1e20;
    pre_dist_wp = 1e20;
    current_delta = 5;

    //最小探索
    int count = 0;
    bool is_pre_p_node_contacted_wall = false;
    for(int i=open_list.size()-1; i>=0; i--)                                            //すべての子ノードを探索
    {
        if(map_grid[pre_p_Node.x][pre_p_Node.y] != 0 || map_grid[pre_p_Node.x+1][pre_p_Node.y-1] != 0 || map_grid[pre_p_Node.x-1][pre_p_Node.y+1] != 0 || map_grid[pre_p_Node.x-1][pre_p_Node.y-1] != 0)
            is_pre_p_node_contacted_wall = true;

        dist_wp = std::sqrt(std::pow(open_list[i].x - x_waypoint, 2) + std::pow(open_list[i].y - y_waypoint, 2));
        if(is_low_cost(i) && is_contact(i))
        {
            // std::cout << "test lc" << std::endl;
            if(dist_wp <= pre_dist_wp)
            {
                // std::cout << "test dist" << std::endl;
                if(x_waypoint != pre_p_Node.x && y_waypoint != pre_p_Node.y)   //斜め移動の処理
                {
                    // std::cout << "test dif wp" << std::endl;
                    //移動方向の記録
                    if(open_list[i].x - pre_p_Node.x == 0 && open_list[i].y - pre_p_Node.y == 1) current_delta = 0;
                    else if(open_list[i].x - pre_p_Node.x == 1 && open_list[i].y - pre_p_Node.y == 0) current_delta = 1;
                    else if(open_list[i].x - pre_p_Node.x == 0 && open_list[i].y - pre_p_Node.y == -1) current_delta = 2;
                    else if(open_list[i].x - pre_p_Node.x == -1 && open_list[i].y - pre_p_Node.y == 0) current_delta = 3;

                    // if(is_contacted_wall || is_pre_p_node_contacted_wall)                                              //壁に接しているとき
                    if(is_pre_p_node_contacted_wall)                                              //壁に接しているとき
                    {
                        if(fabs(x_waypoint-open_list[i].x) > fabs(y_waypoint-open_list[i].y))  //進行方向による場合分け
                        {
                            if(open_list[i].x == pre_p_Node.x)
                            {
                                min_f = open_list[i].f;                                         //最小コストの更新
                                pre_dist_wp = dist_wp;                                          //ウェイポイントとの距離の記録
                                p_Node = open_list[i];                                          //親ノードに設定
                                open_list[i].f = 1e10;                                          //Openノードを選択できなくする
                                count = 1;
                                // std::cout << "test ol.x:" << open_list[i].x << " y:" << open_list[i].y << std::endl;
                                // std::cout << "test add 1.1" << std::endl;
                            }
                            else
                            {
                                std::cout << "not x" << std::endl;
                            }
                        }
                        else
                        {
                            if(open_list[i].y == pre_p_Node.y)
                            {
                                min_f = open_list[i].f;                                         //最小コストの更新
                                pre_dist_wp = dist_wp;                                          //ウェイポイントとの距離の記録
                                p_Node = open_list[i];                                          //親ノードに設定
                                open_list[i].f = 1e10;                                          //Openノードを選択できなくする
                                count = 1;
                                // std::cout << "test ol.x:" << open_list[i].x << " y:" << open_list[i].y << std::endl;
                                // std::cout << "test add 1.2" << std::endl;
                            }
                            else
                            {
                                std::cout << "not y" << std::endl;
                            }
                        }
                    }
                    else if(current_delta != pre_delta)
                    {
                        min_f = open_list[i].f;                                         //最小コストの更新
                        pre_dist_wp = dist_wp;                                          //ウェイポイントとの距離の記録
                        p_Node = open_list[i];                                          //親ノードに設定
                        open_list[i].f = 1e10;                                           //Openノードを選択できなくする
                        count = 1;
                        // std::cout << "test ol.x:" << open_list[i].x << " y:" << open_list[i].y << std::endl;
                        // std::cout << "test add 0" << std::endl;
                    }
                    else
                        std::cout << "error in delta" << std::endl;
                }
                else
                {
                    min_f = open_list[i].f;
                    pre_dist_wp = dist_wp;
                    p_Node = open_list[i];      //親ノードに設定
                    open_list[i].f = 1e10;
                    count = 1;
                    // std::cout << "test add 2" << std::endl;
                }
            }
        }
    }
    if(current_delta == 0) pre_delta = 0;
    else if(current_delta == 1) pre_delta = 1;
    else if(current_delta == 2) pre_delta = 2;
    else if(current_delta == 3) pre_delta = 3;

    //エラー処理
    if(min_f >= 1e10)
    {
        std::cout << "Error in set_pNode\n" << std::endl;
        sleep(3);
        exit;
    }
}
//子ノードの設定
void AStar::set_kNode()
{
    int kn_size = k_Node.size();                            //子ノードリストの大きさを保存
    //子ノードリストを空にする
    for(int i=0; i<kn_size; i++)
    {
        k_Node.pop_back();
    }
    for(int i=0; i<delta.size(); i++)
    {
        k_Node.push_back(init_Node);
    }

    //子ノードの設定
    for(int i=0; i<delta.size(); i++)
    {
        k_Node[i].x = p_Node.x + delta[i][0];               //子ノードの設定
        k_Node[i].y = p_Node.y + delta[i][1];
        //子ノードのコスト求める
        if(map_grid[k_Node[i].x][k_Node[i].y] == 0)        //障害物が存在したらg値大
        {
            k_Node[i].g = p_Node.g + 1;
        }
        else
        {
            k_Node[i].g = wall_cost;
        }
        k_Node[i].f = heuristic[k_Node[i].x][k_Node[i].y] + k_Node[i].g;
    }
    close_list.push_back(p_Node);                           //親ノードをCloseリストに移動
    //test
    // show_NodeList(k_Node);
    // std::cout << std::endl;
}

//Openリストに入っているか調べる
int AStar::is_open(int num)
{
    int i=0;
    for(i=open_list.size()-1; i>=0; i--)
    {
        if( (k_Node[num].x == open_list[i].x) && (k_Node[num].y == open_list[i].y))
        {
            return i;
        }
    }
    return 0;
}

//Closeリストに入っているか調べる
int AStar::is_close(int num)
{
    int i=0;
    for(i=close_list.size()-1; i>=0; i--)
    {
        if((k_Node[num].x == close_list[i].x) && (k_Node[num].y == close_list[i].y))
        {
            return i;
        }
    }
    return 0;
}

//親ノードをパスに追加
void AStar::add_path()
{
    geometry_msgs::PoseStamped path_point;
    path_point.pose.position.x = double((p_Node.x - (row/2)) * resolution);
    path_point.pose.position.y = double((p_Node.y - (col/2)) * resolution);
    path_point.pose.orientation.w = 1;
    wp_path.poses.push_back(path_point);
    std::cout << "test add x:" << p_Node.x << " y:" << p_Node.y << std::endl;
}

//ノードのコストを更新し、親ノードを記録する
void AStar::update_cost()
{
    is_updated = false;
    for(int i=0; i<k_Node.size(); i++)
    {                                             //すべての子ノードについて
        for(int j=0; j<open_list.size() && j<close_list.size(); j++)
        {
            if(!is_close(i)==0)                                     //Closeリストに入っているか調べる
            {
                if(k_Node[i].f < close_list[is_close(i)].f)
                {
                    close_list[j].f = k_Node[i].f;                                  //コストが小さければ更新
                    is_updated = true;
                }
            }
            else if(!is_open(i)==0)
            {                                                       //Openリストに入っているか調べる
                if(k_Node[i].f < open_list[is_open(i)].f)           //コストが小さければ更新
                {
                    open_list[j].f = k_Node[i].f;
                    is_updated = true;
                }
            }else
            {
                open_list.push_back(k_Node[i]);                                     //リストになければ追加
                is_updated = true;
            }
        }
    }
    if(is_updated) add_path(); //親ノードを保存
}

//作成したパスをglobal_pathに追加
void AStar::make_path()
{
    geometry_msgs::PoseStamped path_point;
    path_point.pose.position.x = double((p_Node.x - (row/2)) * resolution);
    path_point.pose.position.y = double((p_Node.y - (col/2)) * resolution);
    path_point.pose.orientation.w = 1;
    wp_path.poses.push_back(path_point);
    // std::cout << "test add path_point.x:" << path_point.pose.position.x << " y:" << path_point.pose.position.y << "\n" << std::endl;

    global_path.poses.insert(global_path.poses.end(), wp_path.poses.begin(), wp_path.poses.end());

    pub_wp_path.publish(wp_path);
}

//A*を実行
void AStar::astar_process()
{
    std::cout << "-----astar_process will begin-----\n" << std::endl;

    set_waypoint();                             //ウェイポイントを作成
    std::cout << "test wp.size:" << waypoint.size() << std::endl;

    //それぞれのウェイポイント間の経路を作成
    for(int phase = 1; phase < waypoint.size(); phase++)
    {
        is_reached = false;                      //is_reachedの初期化
        make_heuristic(phase);                  //heuristic関数の作成
        set_NodeList(open_list);                //openリストcloseリストの初期化
        set_NodeList(close_list);
        wp_path.poses.clear();                  //ウェイポイント間のパスの初期化
        x_waypoint = waypoint[phase][0];    //ウェイポイントの保存
        y_waypoint = waypoint[phase][1];
        std::cout << "test wp.x:" << x_waypoint << " y:" <<  y_waypoint << std::endl;

        //スタートノードをOpenリストに格納
        origin.x = waypoint[phase-1][0];
        origin.y = waypoint[phase-1][1];
        origin.g = 0;
        origin.f = heuristic[origin.x][origin.y] + origin.g;
        open_list.push_back(origin);
        p_Node = origin;
        std::cout << "test origin.x:" << origin.x << " y:" << origin.y << " f:" << origin.f << "\n" << std::endl;

        while(!is_reached)
        {                                              //ウェイポイントに到達するまで
            set_pNode();                                                //親ノードを設定
            if(p_Node.x == x_waypoint && p_Node.y == y_waypoint)
            {       //親ノードとゴールノードが一致したらis_reachedをTrueに
                is_reached = true;
                way_point.pose.position.x = x_waypoint;
                way_point.pose.position.y = y_waypoint;
                pub_wp.publish(way_point);
                std::cout << "reached waypoint" << std::endl;
            }
            else
            {
                set_kNode();                                            //子ノードを設定
                update_cost();                                           //コストを更新し、ノードをパスに追加
            }
        }
        make_path();                 //ウェイポイント間の経路をつなぐ
        std::cout << "test fin phase:" << phase << std::endl;
        std::cout << std::endl;
        // sleep(1);
    }
}//A*process

void AStar::process()
{
    std::cout << "\n-----GlobalPathPlanner will begin-----\n" << std::endl;
    ros::Rate loop_rate(hz);
    while(ros::ok())
    {
        if(map_checker && !path_checker)
        {
            astar_process();
            // global_path.header.frame_id = "map";
            pub_path.publish(global_path);
            path_checker = true;
        }
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
