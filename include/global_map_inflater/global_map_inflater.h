#ifndef GLOBAL_MAP_INFLATER_H
#define GLOBAL_MAP_INFLATER_H

#include<ros/ros.h>
#include "nav_msgs/OccupancyGrid.h"

class Global_Map_Inflater
{
        public:
             Global_Map_Inflater();
             void process();

        private:
             int row_;                                               //元データ
             int column_;                                            //元データの列
             int hz_;                                                //更新周期

             void callback(const nav_msgs::OccupancyGrid::ConstPtr&);//callback関数
             void map2array();                                       //map情報を二次元配列へ格納する関数
             void obstacle_expand();                                 //障害物を拡張する関数
             void remap();                                           //配列情報からmap情報へ戻す関数

             ros::NodeHandle nh;
             ros::NodeHandle private_nh;
             ros::Subscriber original_map;                           //元々のマップを受け取る
             ros::Publisher update_map;                              //更新したマップを送る
             nav_msgs::OccupancyGrid origin_map;                     //元々のマップを使うメッセージ
             nav_msgs::OccupancyGrid new_map;                        //更新したマップを使うメッセージ
             std::vector<std::vector<int>> grid_map;                 //メッセージの処理用の配列
             std::vector<std::vector<int>> copy_grid_map;            //メッセージの処理用の配列
};
#endif
