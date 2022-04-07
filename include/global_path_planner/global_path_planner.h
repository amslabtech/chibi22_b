#ifndef GLOBAL_PATH_PLANNER_H
#define GLOBAL_PATH_PLANNER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <vector>

struct Node{
    double x;
    double y;
    int g;
    int f;
};

class AStar{
    public:
        AStar();
        void process();

    private:
        void set_waypoint();                                            //ウェイポイントの作成
        void set_goal();                                                //ゴールの作成
        void thick_wall();                                              //障害物の当たり判定大きく
        void make_heuristic(int Phase);                                 //heuristic関数作成
        void astar_process();                                           //A*やる
        void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &);   //受け取ったデータをマップに変換
        void make_testMap();                                            //テスト用マップ作成

        void set_NodeList(std::vector<Node> &);                         //ノードリストの初期化
        void show_NodeList(std::vector<Node>);                          //ノードリスト確認
        void add_path();                                                //ウェイポイント間のパスに追加
        void make_path();                                               //ウェイポイント間のパスをつなぐ
        int  is_close(int num);                                         //Closeリストに入っているか
        int  is_open(int num);                                          //Openリストに入っているか
        void set_pNode();                                               //親ノード探索
        void set_kNode();                                               //子ノード探索
        void update_cost();                                             //コスト更新、親ノード記録

        //map作成で使用
        int    hz;
        int    row;
        int    col;
        int    phase;
        double resolution;
        double wall_cost;                                               //障害物のコスト設定
        double map_origin_x;
        double map_origin_y;
        Node   init_Node = {0, 0, 0, 0};                                //初期化ノード
        Node   origin;                                                  //start位置
        std::vector<std::vector<Node>> node_map;                        //ノードの情報を保存したマップ
        std::vector<std::vector<int>> map_grid;                         //map格納
        std::vector<std::vector<int>> occu_map_grid;                    //test
        std::vector<std::vector<int>> heuristic;                        //heuristic関数格納
        std::vector<std::vector<int>> waypoint;                         //waypoint格納
        std::vector<std::vector<int>> delta = {                         //子ノード設定用
            {0,1},
            {0,-1},
            {-1,0},
            {1,0},
        };

        //OpenリストとCloseリストの宣言
        std::vector<Node> open_list;
        std::vector<Node> close_list;

        //ゴールの座標
        int    x_waypoint;
        int    y_waypoint;

        //コストの宣言
        int    g_value;
        int    h;
        int    f_value;

        //親ノードと子ノードの宣言
        Node   p_Node;
        Node   pre_p_Node;
        std::vector<Node> k_Node;

        //set_pNode内で使用
        double min_f;

        //updateCost内で使用
        bool   is_updated;

        //各プロセスが為されているか
        bool   map_checker;
        bool   wall_checker;
        bool   is_reached;
        bool   path_checker;

        ros::NodeHandle nh;
        ros::NodeHandle private_nh;
        ros::Subscriber sub_map;
        ros::Subscriber sub_map_test;
        ros::Publisher pub_path;
        ros::Publisher pub_wp;
        ros::Publisher pub_wp_path;
        ros::Publisher pub_wall;

        nav_msgs::OccupancyGrid map;
        nav_msgs::OccupancyGrid map_data;       //map格納
        nav_msgs::OccupancyGrid wall_map;       //障害物確認用
        nav_msgs::Path wp_path;                 //ウェイポイント間のパス
        nav_msgs::Path global_path;             //出力する経路
        geometry_msgs::PoseStamped way_point;   //ウェイポイントの情報
};
#endif
