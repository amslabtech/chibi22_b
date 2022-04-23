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
    double g;
    double f;
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

        //path作成で使用
        void set_NodeList(std::vector<Node> &);                         //ノードリストの初期化
        void show_NodeList(std::vector<Node>);                          //ノードリスト確認
        void add_path();                                                //ウェイポイント間のパスに追加
        void make_path();                                               //ウェイポイント間のパスをつなぐ
        int  is_close(int num);                                         //Closeリストに入っているか
        int  is_open(int num);                                          //Openリストに入っているか
        int  set_current_delta(int);                                    //移動方向の記録
        bool is_contact(int);                                           //親ノードと接しているか
        bool is_low_cost(int);                                          //コストの比較
        bool is_contacted_wall();                                       //壁に接しているか
        bool is_neared();                                               //距離の比較
        void set_pNode();                                               //親ノード探索
        void set_kNode();                                               //子ノード探索
        void update_cost();                                             //コスト更新、親ノード記録
        void update_p_node(int);                                        //親ノード候補の作成

        //map作成で使用
        int    hz_;
        int    row_;
        int    col_;
        int    phase_;
        double resolution_;
        double wall_cost_;                                               //障害物のコスト設定
        double dist_wp_;
        double pre_dist_wp_;
        int    current_delta_;
        int    pre_delta_;
        Node   init_Node_ = {0, 0, 0, 0};                                //初期化ノード
        Node   origin_;                                                  //start位置
        std::vector<std::vector<int>> map_grid_;                         //map格納
        std::vector<std::vector<int>> heuristic_;                        //heuristic関数格納
        std::vector<std::vector<int>> waypoint_;                         //waypoint格納
        std::vector<std::vector<int>> delta_ = {                         //子ノード設定用
            {0,1},
            {0,-1},
            {-1,0},
            {1,0},
        };

        //OpenリストとCloseリストの宣言
        std::vector<Node> open_list_;
        std::vector<Node> close_list_;

        //ゴールの座標
        int    x_waypoint_;
        int    y_waypoint_;

        //親ノードと子ノードの宣言
        Node   p_Node_;
        Node   pre_p_Node_;
        std::vector<Node> k_Node_;

        //set_pNode内で使用
        double min_f_;

        //updateCost内で使用
        bool   is_updated_;

        //各プロセスが為されているか
        bool   map_checker_;
        bool   wall_checker_;
        bool   is_reached_;
        bool   path_checker_;

        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        ros::Subscriber sub_map_;
        ros::Publisher pub_path_;
        ros::Publisher pub_wp_path_;

        nav_msgs::OccupancyGrid map_;
        nav_msgs::OccupancyGrid map_data_;       //map格納
        nav_msgs::Path wp_path_;                 //ウェイポイント間のパス
        nav_msgs::Path global_path_;             //出力する経路
        geometry_msgs::PoseStamped path_point_;
};
#endif
