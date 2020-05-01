#include "mainrun.h"

namespace wjz_controller{

    MainRun::MainRun(){
        Main_Running = true;
        seq = 0;
        tf_seq = 0;
    }

    MainRun::~MainRun(){}

    void MainRun::Read_From_File(){
        Pose_6d tmp_pose;
        R8 tmp;
        std::ifstream infile;
        infile.open(file_path.c_str());
        if(!infile.is_open()){
            printf("open file error!");
        }
        while(infile){
            infile >> tmp;
            infile >> tmp_pose.x;
            infile >> tmp_pose.y;
            infile >> tmp;
            infile >> tmp_pose.yaw;
            infile >> tmp;
            infile >> tmp;
            infile >> tmp_pose.v;
            infile >> tmp_pose.a;
            path_read.push_back(tmp_pose);
        }
        infile.close();
    }

    void MainRun::ROS_Param_Read(){
        private_nh_.param<std::string>("openfilepath", file_path, "/D_ubuntu/catkin_ws/src/WJZ_Controller/src/b4-b2-w.txt");
    }

    void MainRun::Main_Function(){
        ROS_Param_Read();
        Read_From_File();
        nav_msgs::Path path;
        path.poses.resize(path_read.size());

        Control_Param param;
        //0.for all
        param.min_index = 0;
        param.max_index = 50;
        param.dt = 0.1;
        param.mode = PID_ACM;
        param.max_v = 2;
        //1.for PID_Diff_Robot
        param.Pv = 5;
        param.Iv = 0;
        param.Dv = 0.1;
        param.Pw = 5;
        param.Iw = 0;
        param.Dw = 0.1;
        param.max_w = 0.45;
        param.target_inc = 10;
        //2.for ACM model
        controller.Set_Wheel_Base(2.0);
        //3.for PID_ACM model
        param.P_lon = 10;
        param.I_lon = 0;
        param.D_lon = 0.05;
        param.P_lat = 1;
        param.I_lat = 0;
        param.D_lat = 0.01;
        param.max_a = 1;
        param.max_steer = 0.45;
        param.ag = 0.8;
        param.bg = 0.1;
        param.review_num = 20;

        //1.init path
        R8 ox = path_read[0].x;
        R8 oy = path_read[0].y;
        for(U4 i = 0;i<path_read.size();i++){
            path_read[i].x -= ox;
            path_read[i].y -= oy;
            path_read[i].yaw = ENU2RC(path_read[i].yaw);
            path.poses[i].pose.position.x = path_read[i].x;
            path.poses[i].pose.position.y = path_read[i].y;
            path.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
        }

        //2.loop
        Pose_6d now_pose;
        now_pose.x = 0.0;
        now_pose.y = 0.0;
        now_pose.yaw = path_read[0].yaw;
        now_pose.v = 3;
        nav_msgs::Odometry odom_now;
        geometry_msgs::TransformStamped odom_trans;
        tf::TransformBroadcaster odom_broadcaster;
        while(Main_Running){
            ros::Time now_time = ros::Time::now();
            //1.pub path
            path.header.stamp = now_time;
            path.header.frame_id = "/path";
            path.header.seq = seq;
            path_pub.publish(path);

            //2.update control
            if(!controller.path_OK){
                controller.Get_Control(now_pose,path_read,param);
                now_pose = controller.Simulation_Update(now_pose,param);
                printf("now:%f,%f,%f,%f\n",now_pose.x,now_pose.y,now_pose.yaw,now_pose.v);
            }

            //3.pub odom
            geometry_msgs::Quaternion th_q = tf::createQuaternionMsgFromYaw(now_pose.yaw);
            odom_trans.header.stamp = now_time;
            odom_trans.header.frame_id = "/odom";
            odom_trans.child_frame_id = "/base_link";
            odom_trans.transform.translation.x = now_pose.x;
            odom_trans.transform.translation.y = now_pose.y;
            odom_trans.transform.translation.z = 0.0;
            odom_trans.transform.rotation = th_q;
            odom_broadcaster.sendTransform(odom_trans);

            odom_now.header.stamp = now_time;
            odom_now.header.seq = seq++;
            odom_now.header.frame_id = "/odom";
            odom_now.pose.pose.position.x  = now_pose.x;
            odom_now.pose.pose.position.y  = now_pose.y;
            odom_now.pose.pose.position.z  = 0.0;
            odom_now.pose.pose.orientation = th_q;
            odom_pub.publish(odom_now);

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    void MainRun::tf_publish_function(R8 time_interval){
        ros::Rate r(1.0 / time_interval);
        geometry_msgs::TransformStamped path_trans;
        geometry_msgs::Quaternion th_q = tf::createQuaternionMsgFromYaw(0.0);
        while(ros::ok()){
            path_trans.header.stamp = ros::Time::now() + ros::Duration(time_interval);
            path_trans.header.seq   = tf_seq++;
            path_trans.header.frame_id = "/odom";
            path_trans.child_frame_id = "/path";
            path_trans.transform.translation.x = 0.0; //you had known
            path_trans.transform.translation.y = 0.0;
            path_trans.transform.translation.z = 0.0;
            path_trans.transform.rotation = th_q;
            tf::TransformBroadcaster global_map_broadcaster;
            global_map_broadcaster.sendTransform(path_trans);
            r.sleep();
        }
    }

    void MainRun::Run() {
        path_pub  = node_.advertise<nav_msgs::Path>("/path",10,true);
        odom_pub  = node_.advertise<nav_msgs::Odometry>("/odom",10,true);
        Main_thread = new std::thread(&MainRun::Main_Function,this);
        tf_thread = new std::thread(std::bind(&MainRun::tf_publish_function,this,0.005));
    }

}
