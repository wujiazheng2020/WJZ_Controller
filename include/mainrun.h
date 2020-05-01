/*
 * Copyright 2020 WJZ_Controller, Jiazheng Wu
 *
 * Licensed under the MIT License
 */

#ifndef WJZ_CONTROLLER_MAINRUN_H
#define WJZ_CONTROLLER_MAINRUN_H

#include "controller.h"
#include <thread>
#include <fstream>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

namespace wjz_controller{

    class MainRun{
    public:
        MainRun();
        ~MainRun();
        void Run();
        void Main_Function();
        void tf_publish_function(R8 time_interval);;
        void Read_From_File();
        void ROS_Param_Read();

    private:
        ros::NodeHandle node_;
        ros::NodeHandle private_nh_;
        ros::Publisher path_pub;
        ros::Publisher odom_pub;

        X1 Main_Running;
        X1 seq,tf_seq;
        std::string file_path;
        std::vector<Pose_6d> path_read;
        Controller controller;

        std::thread *Main_thread;
        std::thread *tf_thread;
    };

}

#endif //WJZ_CONTROLLER_MAINRUN_H
