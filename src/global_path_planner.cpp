#include "global_path_planner.h"

//角度の情報
//Mapの読み込み・出力方法

AStar::AStar():private_nh("~"){
    private_nh.param("hz", hz, {10});
    private_nh.param("map_checker", map_checker,{false});
    //private_nh.param("wall_checker", wall_checker, {false});
    private_nh.param("wall_cost", wall_cost, {1e10});

    sub_map = nh.subscribe("map", 10, &AStar::set_map, this);
    pub_path = nh.advertise<nav_msgs::Path>("/path", 1);        //出力するパス
    pub_path_set = nh.advertise<geometry_msgs::PointStamped>("open_set_rviz", 1);//rviz用
}

void AStar::set_map(const nav_msgs::OccupancyGrid::ConstPtr &msg){
    if(map_checker){
        std::cout << "Error int set_map" << std::endl;
        exit;
    }else{
        map = *msg;

        row = map.info.height;
        col = map.info.width;
        map_grid = std::vector<std::vector<int>>(row, std::vector<int>(col,0));

        //マップデータを変換
        for(int i=0;i<row;i++){
            for(int j=0;j<col;j++){
                map_grid[i][j] = map.data[i+j*row];
            }
        }
        origin.x = map.info.origin.position.x;
        origin.y = map.info.origin.position.y;

        map_checker = true;
        std::cout << "set_map finished" << std::endl;
    }
}

// void AStar::thick_wall(){
//     if(wall_checker){
//         for(int i=0; i<row; i++){
//             for(int j=0; j<col; j++){
//                 //障害物を判別して拡張
//             }
//         }
//     }
// }

void AStar::set_waypoint(){
    //ウェイポイントの座標を設定
    waypoint = {
        {0,0},
        {0,0},
        {0,0},
        {0,0},
        {0,0},
        {0,0},
    };
}

void AStar::set_NodeList(std::vector<Node> list){
    for(int i=0; i<list.size(); i++){
        list[i].g = 0;
        list[i].f = 0;
        list[i].x = 0;
        list[i].y = 0;
    }
}

void AStar::make_heuristic(){
    heuristic = std::vector<std::vector<int>>(row, std::vector<int>(col,0));
    //heuristic.assign(row, (col, 0));
    for(int i=0; i<row; i++){
        for(int j=0; j<col; j++){
            //ゴールからの距離で設定
            heuristic[i][j] = abs(waypoint[phase][0] - i) + abs(waypoint[phase][1] - j);
        }
    }
}

void AStar::set_pNode(){
    //Openリストの中でコストの最も小さいものを探索し、親ノードに設定
    p_Node.x = 0;
    p_Node.y = 0;
    p_Node.g = 0;
    p_Node.f = 0;
    min_f = 1e10;
    for(int i=0; i<row; i++){       //最小探索
        if(open_list[i].f < min_f){
            min_f = open_list[i].f;
            p_Node = open_list[i];  //親ノードに設定
        }
    }
    if(min_f == 1e+10){
        std::cout << "Error in set_pNode" << std::endl;
        exit;
    }
    std::cout << "p_Node:" << p_Node.g << std::endl;
}

void AStar::set_kNode(){
    std::cout << "p_Node:" << p_Node.g << std::endl;
    delta = {
        {0,1},
        {0,-1},
        {-1,0},
        {1,0},
    };
    //子ノードの初期化
    for(int i=0; i<delta.size(); i++){
        //k_Node[i] = {0,0,0,0};
        k_Node[i].x = 0;
        k_Node[i].y = 0;
        k_Node[i].g = 0;
        k_Node[i].f = 0;
    }
    //子ノードの設定
    //子ノードのコスト求める
    for(int i=0; i<delta.size(); i++){
        k_Node[i].x = p_Node.x + delta[i][0];
        k_Node[i].y = p_Node.y + delta[i][1];
        //障害物が存在したらg値大
        if(map_grid[k_Node[i].x][k_Node[i].y] >= 1){
            k_Node[i].g = p_Node.g + 1;
        }else{
            k_Node[i].g = wall_cost;
        }
        k_Node[i].f = heuristic[k_Node[i].x][k_Node[i].y];
    }
    //親ノードをCloseリストに移動
    close_list.push_back(p_Node);
}

bool AStar::isOpen(int num){
    for(int i=0; i<open_list.size(); i++){
        if( (k_Node[num].x == open_list[i].x) && (k_Node[num].y == open_list[i].y)){
            return true;
        }
    }
    return false;
}

bool AStar::isClose(int num){
    for(int i=0; i<close_list.size(); i++){
        if( (k_Node[num].x == close_list[i].x) && (k_Node[num].y == close_list[i].y)){
            return true;
        }
    }
    return false;
}

void AStar::addPath(){                          //親ノードをパスに追加
    geometry_msgs::PoseStamped path_point;
    path_point.pose.position.x = p_Node.x;
    path_point.pose.position.y = p_Node.y;
    wp_path.poses.push_back(path_point);
}

void AStar::updateCost(){
    for(int i=0; i<k_Node.size(); i++){                                             //すべての子ノードについて
        for(int j=0; j<open_list.size() && j<close_list.size(); j++){
            if(isClose(i)){
                if(k_Node[i].f < close_list[j].f) close_list[j].f = k_Node[i].f;    //コストが小さければ更新
            }else if(isOpen(i)){
                if(k_Node[i].f < open_list[j].f) open_list[j].f = k_Node[i].f;
            }else{
                open_list.push_back(k_Node[i]);                                     //リストになければ追加
            }
            addPath();                                                              //親ノードを保存
        }
    }
}

void AStar::makePath(){
    //作成したパスをglobal_pathに移動
    for(int i=0; i<wp_path.poses.size(); i++){
        global_path.poses.insert(global_path.poses.end(), wp_path.poses.begin(), wp_path.poses.end());
    }
    std::cout << "makePath finished" << std::endl;
}

void AStar::AStarProcess(){
    //openリストcloseリストの初期化
    set_NodeList(open_list);
    set_NodeList(close_list);

    //それぞれのウェイポイント間の経路を作成
    for(int phase = 1; phase < waypoint.size(); phase++){
        make_heuristic();
        int x_waypoint = waypoint[phase][0];
        int y_waypoint = waypoint[phase][1];

        //スタートノードをOpenリストに格納
        origin.x = waypoint[phase-1][0];
        origin.y = waypoint[phase-1][1];
        origin.g = 0;
        origin.f = heuristic[origin.x][origin.y] + origin.g;
        open_list.push_back(origin);

        while(!isReached){
            //Openリストの中でコストの最も小さいものを探索
            //親ノードを設定
            set_pNode();
            //親ノードとゴールノードが一致したらisReachedをTrueに
            if(p_Node.x = waypoint[phase][0] && p_Node.y == waypoint[phase][1]){
                isReached = true;
            }else{
                //親ノードに隣接するノードを子ノードに
                //親ノードをClose
                //子ノードのコスト求める
                set_kNode();
                //Closeリストに存在するか調べる
                //Openリストに存在するか調べる
                //コスト更新
                //親ノード記録
                updateCost();
            }
        }
    }
    //ウェイポイント間の経路をつなぐ
    makePath();
}//A*process

void AStar::process(){
    ros::Rate loop_rate(hz);
    while(ros::ok()){
        if(map_checker && !path_checker){
//            thick_wall();
//            set_waypoint();
            AStarProcess();

            pub_path.publish(global_path);
            path_checker = true;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char** argv){

    ros::init(argc, argv, "Global_Path_Planner");
    AStar astar;
    std::cout << "Global_Path_Planner will begin" << std::endl;
    astar.process();

    return 0;
}


