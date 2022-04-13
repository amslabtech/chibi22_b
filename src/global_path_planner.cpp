#include "global_path_planner.h"

//Todo
//角度の情報
//処理の軽量化

AStar::AStar():private_nh("~")
{
    private_nh.param("hz", hz, {10});
    private_nh.param("map_checker", map_checker, {false});
    private_nh.param("is_reached", is_reached, {false});
    private_nh.param("wall_cost", wall_cost, {1e6});

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
            }
        }

        origin.x = map_data.info.origin.position.x;                                      //原点の保存
        origin.y = map_data.info.origin.position.y;

        global_path.header.frame_id = "map";
        wp_path.header.frame_id = "map";
        way_point.header.frame_id = "map";

        map_checker = true;
        std::cout << "row:" << row << " col:" << col << std::endl;
        std::cout << "set_map finished\n" << std::endl;
        sleep(1);
    }
}

void AStar::make_testMap()
{
    row = 2000;
    col = 2000;
    map_grid = std::vector<std::vector<int>>(row, std::vector<int>(col, 0));

    map_checker = true;
    std::cout << "test set_map finished\n" << std::endl;
}

//ウェイポイントの座標を設定
void AStar::set_waypoint()
{
    waypoint = {
        {1950,2520},
        {2595,2520},
        {2595,2820},
        {1950,2820},
        // {1930,2670},
        {1950,2520},
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
        if(open_list[i].f < min_f && open_list[i].f >= 0)
            return true;
        else
            return false;
}

bool AStar::is_contact(int i){
        if(fabs(open_list[i].x-pre_p_Node.x) <= 1 && fabs(open_list[i].y-pre_p_Node.y) <= 1)
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
    p_Node.f = 1e6;
    min_f = 1e+10;
    dist_wp = 1e20;
    pre_dist_wp = 1e20;
    current_delta = 5;

    //最小探索
    for(int i=open_list.size()-1; i>=0; i--)
    {
        dist_wp = std::sqrt(std::pow(open_list[i].x - x_waypoint, 2) + std::pow(open_list[i].y - y_waypoint, 2));
        if(is_low_cost(i) && is_contact(i))
        {
            if(dist_wp <= pre_dist_wp)
            {
                if(x_waypoint != pre_p_Node.x && y_waypoint != pre_p_Node.y)
                {
                    if(open_list[i].x - pre_p_Node.x == 0 && open_list[i].y - pre_p_Node.y == 1) current_delta = 0;
                    else if(open_list[i].x - pre_p_Node.x == 1 && open_list[i].y - pre_p_Node.y == 0) current_delta = 1;
                    else if(open_list[i].x - pre_p_Node.x == 0 && open_list[i].y - pre_p_Node.y == -1) current_delta = 2;
                    else if(open_list[i].x - pre_p_Node.x == -1 && open_list[i].y - pre_p_Node.y == 0) current_delta = 3;
                    std::cout << "test current_delta:" << current_delta << std::endl;
                    if(current_delta != pre_delta)
                    {
                        std::cout << "test dist:" << dist_wp << " pre_dist_wp:" << pre_dist_wp << std::endl;
                        min_f = open_list[i].f;
                        pre_dist_wp = dist_wp;
                        p_Node = open_list[i];  //親ノードに設定
                        open_list[i].f = 1e6;
                        std::cout << "test p_x:" << p_Node.x << " y:" << p_Node.y << std::endl;
                    }
                }
                else
                {
                    std::cout << "test dist:" << dist_wp << " pre_dist_wp:" << pre_dist_wp << std::endl;
                    min_f = open_list[i].f;
                    pre_dist_wp = dist_wp;
                    p_Node = open_list[i];  //親ノードに設定
                    open_list[i].f = 1e6;
                    std::cout << "test p_x:" << p_Node.x << " y:" << p_Node.y << std::endl;
                }
            }
        }
    }
    if(current_delta == 0) pre_delta = 0;
    else if(current_delta == 1) pre_delta = 1;
    else if(current_delta == 2) pre_delta = 2;
    else if(current_delta == 3) pre_delta = 3;

    //エラー処理
    if(min_f >= 1e6)
    {
        std::cout << "Error in set_pNode" << std::endl;
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
        // if(map_grid[k_Node[i].x][k_Node[i].y] == 0){        //障害物が存在したらg値大
        if(map_grid[k_Node[i].x][k_Node[i].y] <= 100)       //test
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
    path_point.header.frame_id = "map";
    wp_path.poses.push_back(path_point);
    std::cout << "test add x:" << p_Node.x << " y:" << p_Node.y << std::endl;
    std::cout << "test add path_point.x:" << path_point.pose.position.x << " y:" << path_point.pose.position.y << "\n" << std::endl;
}

//ノードのコストを更新し、親ノードを記録する
void AStar::update_cost()
{
    is_updated = false;
    for(int i=0; i<k_Node.size(); i++)
    {                                             //すべての子ノードについて
        for(int j=0; j<open_list.size() && j<close_list.size(); j++)
        {
            if(!is_close(i)==0)
            {                                                     //Closeリストに入っているか調べる
                if(k_Node[i].f < close_list[is_close(i)].f)
                {
                    close_list[j].f = k_Node[i].f;                                  //コストが小さければ更新
                    is_updated = true;
                }
            }
            else if(!is_open(i)==0)
            {                                                //Openリストに入っているか調べる
                if(k_Node[i].f < open_list[is_open(i)].f)
                {                           //コストが小さければ更新
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
    path_point.header.frame_id = "map";
    wp_path.poses.push_back(path_point);
    std::cout << "test add path_point.x:" << path_point.pose.position.x << " y:" << path_point.pose.position.y << "\n" << std::endl;

    global_path.poses.insert(global_path.poses.end(), wp_path.poses.begin(), wp_path.poses.end());

    wp_path.header.frame_id = "map";
    pub_wp_path.publish(wp_path);
}

//A*を実行
void AStar::astar_process()
{
    std::cout << "-----astar_process will begin-----\n" << std::endl;

    set_waypoint();                             //ウェイポイントを作成

    //それぞれのウェイポイント間の経路を作成
    std::cout << "test wp.size:" << waypoint.size() << std::endl;
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
                way_point.header.frame_id = "map";
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
        sleep(1);
    }
}//A*process

void AStar::process()
{
    std::cout << "\n-----Global_Path_Planner will begin-----\n" << std::endl;
    ros::Rate loop_rate(hz);
    while(ros::ok())
    {
        if(map_checker && !path_checker)
        {
            astar_process();
            global_path.header.frame_id = "map";
            pub_path.publish(global_path);
            pub_wall.publish(wall_map);
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
