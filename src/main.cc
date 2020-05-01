#include "mainrun.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "WJZ_Controller");
    wjz_controller::MainRun Main_Running;
    Main_Running.Run();
    ros::spin();
    return 0;
}
