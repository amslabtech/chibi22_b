#ifndef OBSTACLE_EXPANDER_H
#define OBSTACLE_EXPANDER_H
#include<ros/ros.h>
#include "nav_msgs/OccupancyGrid.h"

// ====クラス====
class Obstacle_Expander
{
public:
    Obstacle_Expander();                                  //デフォルトコンストラクタ
    void process();

private:
// ----変数----
    int row_;                                               //元データ
    int column_;                                            //元データの列
    int hz_;                                                //更新周期
    bool map_check_;                                        //nodeの停止のためのbool
// ----関数----
    void callback(const nav_msgs::OccupancyGrid::ConstPtr&);//callback関数
    void map2array();                                       //map情報を二次元配列へ格納する関数
    void obstacle_expand();                                 //障害物を拡張する関数
    void remap();                                           //配列情報からmap情報へ戻す関数

// ----NodeHandle----
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;

// ----Subscriber----
    ros::Subscriber original_map;                           //元々のマップを受け取る

// ----Publisher----
    ros::Publisher update_map;                              //更新したマップを送る

// ----ros messege----
    nav_msgs::OccupancyGrid origin_map;                     //元々のマップを使うメッセージ
    nav_msgs::OccupancyGrid new_map;                        //更新したマップを使うメッセージ

// ----二次元配列----
    std::vector<std::vector<int>> grid_map;                 //マップのメッセージ用の配列
    std::vector<std::vector<int>> copy_grid_map;            //マップのメッセージ用の配列
};
#endif
