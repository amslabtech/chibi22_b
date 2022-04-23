#include "local_path_planner/local_path_planner.h"

//===== メイン関数 =====
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "local_path_planner"); // ノードの初期化
    DWA dwa;
    dwa.process();

    return 0;
}