/*
 * Copyright 2020 WJZ_Controller, Jiazheng Wu
 *
 * Licensed under the MIT License
 */

#ifndef WJZ_CONTROLLER_COMMON_H
#define WJZ_CONTROLLER_COMMON_H

#include <cstdlib>
#include <ctime>
#include <cmath>
#include <cassert>
#include <random>
#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Core>
#include <Eigen/QR>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "ros/ros.h"
#include "ros/console.h"
#include "special_define.h"

#define DEG2RAD(X) X*3.1415926/180

namespace wjz_controller {

    class Pose_6d{
    public:
        Pose_6d();
        Pose_6d(const Pose_6d &obj);

        R8 x;   //m
        R8 y;   //m
        R8 yaw; //rad
        R8 v;   //m/s
        R8 w;   //rad/s
        R8 a;   //m/s2
    };

    //0.auxiliary functions
    R8 yaw_sub(R8 yaw1,R8 yaw2);
    R8 yaw_add(R8 yaw1,R8 yaw2);
    R8 get_angle(R8 y1,R8 x1,R8 y0,R8 x0);
    R8 ENU2RC(R8 yaw);//ENU axis to Rectangular Coordinates
}

#endif //WJZ_CONTROLLER_COMMON_H
