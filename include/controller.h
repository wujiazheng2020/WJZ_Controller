/*
 * Copyright 2020 WJZ_Controller, Jiazheng Wu
 *
 * Licensed under the MIT License
 */

#ifndef WJZ_CONTROLLER_CONTROLLER_H
#define WJZ_CONTROLLER_CONTROLLER_H

#include "common.h"

#define PID_Diff 1
#define PID_ACM 2
#define MPC_ACM 3

namespace wjz_controller{
    class Control_Param{
    public:
        Control_Param();
        ~Control_Param();

        //0.for all
        I4 min_index;
        I4 max_index;
        R8 dt;
        X1 mode;
        R8 max_v;

        //1.for PID_Diff_Robot
        R8 Pv,Iv,Dv;
        R8 Pw,Iw,Dw;
        I4 target_inc;
        R8 max_w;

        //2.for PID_ACM Model
        R8 P_lon,I_lon,D_lon;
        R8 P_lat,I_lat,D_lat;
        R8 ag,bg;
        I4 review_num;
        R8 max_a,max_steer;
    };

    class Controller{
    public:
        Controller();
        ~Controller();

        //0.auxiliary functions and variable
        X1 path_OK;
        void Set_Wheel_Base(R8 wb);
        Pose_6d Simulation_Update(Pose_6d &now_pose,Control_Param &param);
        //for MPC only
        R8 polyeval(Eigen::VectorXd coeffs, R8 x);
        Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,I4 order);

            //1.main function
        void Get_Control(Pose_6d &now_pose,std::vector<Pose_6d> &refer_line,Control_Param &param);

        //2.for diff robot PID
        void PID_Diff_Robot(Pose_6d &now_pose,std::vector<Pose_6d> &refer_line,Control_Param &param);

        //3.for ACM model like auto car
        //3.1.PID with review ,usually d_a , d_steer should be limited
        void PID_ACM_Model(Pose_6d &now_pose,std::vector<Pose_6d> &refer_line,Control_Param &param);

        //3.2 MPC controller,you should git clone udaicty MPC code:MPC.cpp, MPC.h,ipopt
        void MPC_ACM_Model(Pose_6d &now_pose,std::vector<Pose_6d> &refer_line,Control_Param &param);

    private:
        //0.for all
        I4 now_i;
        //1.for diff robot
        R8 v;
        R8 w;
        R8 D[3],Y[3];
        //2.for auto car
        R8 steer;
        R8 a;
        R8 wheel_base;
    };

}

#endif //WJZ_CONTROLLER_CONTROLLER_H
