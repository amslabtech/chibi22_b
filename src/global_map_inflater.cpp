#include "chibi22_b/global_map_inflater.h"

Global_Map_Inflater::Global_Map_Inflater():private_nh("~"),nh("")
{
    private_nh.param("hz_",hz_,{1});
    private_nh.param("row_",row_,{2000});                                      //マップの行数は適宜設定する
    private_nh.param("column_",column_,{2000});                                //マップの列数は適宜設定する
    update_map = nh.advertise<nav_msgs::OccupancyGrid>("/new_map",1);          //更新したマップをpublish(出版)するノード
    original_map = nh.subscribe("/map",10,&Global_Map_Inflater::callback,this); //元のマップをsubscribe(購読)するノード
}

void Global_Map_Inflater::callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)//callback関数
{
    std::cout<<"callback starts"<<std::endl;
    ROS_INFO("receved original map");
    origin_map = *msg;

    map2array();
    obstacle_expand();
    remap();

    update_map.publish(new_map);
    ROS_INFO("sended new_map");
}

void Global_Map_Inflater::map2array()                                           //マップ情報を配列へ格納する関数
{
    std::cout<<"map2array starts"<<std::endl;
    if(origin_map.data.size() != 0)                                            //map情報が入っているかの確認
    {
      std::cout<<"origin map input OK"<<std::endl;
    }
    else
    {
      std::cout<<"NO input"<<std::endl;
    }

    row = origin_map.info.height;
    column = origin_map.info.width;

    std::cout<<"row"<<row<<std::endl;
    std::cout<<"column"<<column<<std::endl;

    int count = 0;

    for(int i = 0;i < row; i++)
    {
        for(int j = 0;j < column;j++)
        {
            grid_map[i][j] = origin_map.data[count];
            copy_grid_map[i][j] = origin_map.data[count];
            count++;
        }
    }
}

void Global_Map_Inflater::obstacle_expand()                                     //マップの障害物を拡大する
{
    std::cout<<"Obstacle expand starts"<<std::endl;

    for(int i=0;i<row;i++)
    {
        for(int j=3;j<column-3;j++)
        {
            if(copy_grid_map[i][j] == 100)
            {
                grid_map[i][j-1] = 100;
                grid_map[i][j-2] = 100;
                grid_map[i][j-3] = 100;

                grid_map[i][j+1] = 100;
                grid_map[i][j+2] = 100;
                grid_map[i][j+3] = 100;
            }
        }
    }
}

void Global_Map_Inflater::remap()                                               //配列からメッセージとして送信する関数
{
    std::cout<<"remap starts"<<std::endl;
    new_map.header = origin_map.header;
    new_map.header.stamp = ros::Time::now();
    new_map.info = origin_map.info;

    for(int i = 0;i<row;i++)
    {
        for(int j = 0;j<column;j++)
        {
            new_map.data.push_back(grid_map[i][j]);
        }
    }

    if(new_map.data.size() != 0)                                               //map情報が入っているかの確認
    {
      std::cout<<"new map export OK"<<std::endl;
    }
    else
    {
      std::cout<<"new map export NO"<<std::endl;
    }

}

void Global_Map_Inflater::process()                                             //プロセス関数
{
    grid_map = std::vector<std::vector<int>>(row, std::vector<int>(column,0)); //配列の初期化
    copy_grid_map = std::vector<std::vector<int>>(row, std::vector<int>(column,0));
    std::cout<<"process starts"<<std::endl;
    ros::Rate loop_rate(hz);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"global_map_inflater");
    Global_Map_Inflater global_map_inflater;
    std::cout<<"global_map_inflater starts"<<std::endl;
    global_map_inflater.process();
    return 0;
}
