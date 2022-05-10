#include "obstacle_expander/obstacle_expander.h"

Obstacle_Expander::Obstacle_Expander():private_nh("~"),nh("")
{
    private_nh.param("hz_",hz_,{1});
    private_nh.param("row_",row_,{4409});                                      //マップの行数は適宜設定する
    private_nh.param("column_",column_,{4409});                                //マップの列数は適宜設定する
    private_nh.param("map_check",map_check_,{false});
     //マップが送られたかのbool
    update_map = nh.advertise<nav_msgs::OccupancyGrid>("/new_map",1);          //更新したマップをpublish(出版)するノード
    original_map = nh.subscribe("/map",10,&Obstacle_Expander::callback,this);//元のマップをsubscribe(購読)するノード
}

void Obstacle_Expander::callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)//callback関数
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

void Obstacle_Expander::map2array()//マップ情報を配列へ格納する関数
{
    std::cout<<"map2array starts"<<std::endl;
    if(origin_map.data.size() != 0)//map情報が入っているかの確認
    {
      std::cout<<"origin map input OK"<<std::endl;
    }
    else
    {
      std::cout<<"NO input"<<std::endl;
    }

    row_ = origin_map.info.height;
    column_ = origin_map.info.width;

    std::cout<<"row_"<<row_<<std::endl;
    std::cout<<"column_"<<column_<<std::endl;

    int count = 0;

    for(int i = 0;i < row_; i++)
    {
        for(int j = 0;j < column_;j++)
        {
            grid_map[i][j] = origin_map.data[count];
            copy_grid_map[i][j] = origin_map.data[count];
            count++;
        }
    }
}

void Obstacle_Expander::obstacle_expand()//マップの障害物を拡大する
{
    std::cout<<"Obstacle expander starts"<<std::endl;

    for(int i=0;i<row_;i++)
    {
        for(int j=3;j<column_-3;j++)
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

void Obstacle_Expander::remap()//配列からメッセージとして送信する関数
{
    std::cout<<"remap starts"<<std::endl;
    new_map.header = origin_map.header;
    new_map.header.stamp = ros::Time::now();
    new_map.info = origin_map.info;

    for(int i = 0;i<row_;i++)
    {
        for(int j = 0;j<column_;j++)
        {
            new_map.data.push_back(grid_map[i][j]);
        }
    }

    if(new_map.data.size() != 0)//map情報が入っているかの確認
    {
      std::cout<<"new map export OK"<<std::endl;
    }
    else
    {
      std::cout<<"new map export NO"<<std::endl;
    }
    map_check_ = true;

}

void Obstacle_Expander::process()//プロセス関数
{
    grid_map = std::vector<std::vector<int>>(row_, std::vector<int>(column_,0));//配列の初期化
    copy_grid_map = std::vector<std::vector<int>>(row_, std::vector<int>(column_,0));
    std::cout<<"process starts"<<std::endl;
    ros::Rate loop_rate(hz_);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();

        if(map_check_)
        {
            std::cout<<"finish map make"<<std::endl;
            exit(0);
        }

    }
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"obstacle_expander");
    Obstacle_Expander obstacle_expander ;
    std::cout<<"obstacle_expander starts"<<std::endl;
    obstacle_expander.process();
    return 0;
}
