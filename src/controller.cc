#include "controller.h"

namespace wjz_controller{

    Control_Param::Control_Param(){
    }

    Control_Param::~Control_Param(){}

    Controller::Controller(){
        now_i = 0;
        path_OK = false;
        for(U4 i=0;i<3;i++){
            D[i] = 0;
            Y[i] = 0;
        }
    }

    Controller::~Controller(){}

    void Controller::Set_Wheel_Base(R8 wb){
        this->wheel_base = wb;
    }

    Pose_6d Controller::Simulation_Update(Pose_6d &now_pose,Control_Param &param){
        Pose_6d new_pose;
        X1 mode = param.mode;
        R8 dt = param.dt;
        if(mode == PID_Diff){
            new_pose.x   = now_pose.x + v*dt*cos(now_pose.yaw);
            new_pose.y   = now_pose.y + v*dt*sin(now_pose.yaw);
            new_pose.yaw = now_pose.yaw + w*dt;
            new_pose.v   = now_pose.v;
        } else {
            new_pose.x   = now_pose.x + now_pose.v*dt*cos(now_pose.yaw);
            new_pose.y   = now_pose.y + now_pose.v*dt*sin(now_pose.yaw);
            new_pose.yaw = now_pose.yaw + now_pose.v*dt/wheel_base*tan(steer);
            new_pose.v   = now_pose.v + a*dt;
        }

        return new_pose;
    }

    R8 Controller::polyeval(Eigen::VectorXd coeffs, R8 x){
        double result = 0.0;
        for (int i = 0; i < coeffs.size(); i++) {
            result += coeffs[i] * pow(x, i);
        }
        return result;
    }

    Eigen::VectorXd Controller::polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,I4 order){
        assert(xvals.size() == yvals.size());
        assert(order >= 1 && order <= xvals.size() - 1);
        Eigen::MatrixXd A(xvals.size(), order + 1);

        for (int i = 0; i < xvals.size(); i++) {
            A(i, 0) = 1.0;
        }

        for (int j = 0; j < xvals.size(); j++) {
            for (int i = 0; i < order; i++) {
                A(j, i + 1) = A(j, i) * xvals(j);
            }
        }

        auto Q = A.householderQr();
        auto result = Q.solve(yvals);
        return result;
    }

    void Controller::Get_Control(Pose_6d &now_pose,std::vector<Pose_6d> &refer_line,Control_Param &param){
        switch (param.mode) {
            case PID_Diff:
                PID_Diff_Robot(now_pose,refer_line,param);
                break;
            case PID_ACM:
                PID_ACM_Model(now_pose,refer_line,param);
                break;
            case MPC_ACM:
                MPC_ACM_Model(now_pose,refer_line,param);
                break;
            default:
                printf("mode error,check mode!");
                break;
        }

    }

    void Controller::PID_Diff_Robot(Pose_6d &now_pose,std::vector<Pose_6d> &refer_line,Control_Param &param){
        //1.find min index
        I4 line_size = refer_line.size();
        I4 min_i = 0;
        R8 min_dis = 999999999;
        R8 dis;
        I4 start_index = (now_i-param.min_index)<0?0:(now_i-param.min_index);
        I4 end_index = (now_i+param.max_index)>=line_size?line_size-1:(now_i+param.max_index);
        for(U4 i = start_index;i<=end_index;i++){
            dis = (refer_line[i].x - now_pose.x)*(refer_line[i].x - now_pose.x) +
                  (refer_line[i].y - now_pose.y)*(refer_line[i].y - now_pose.y) ;
            if(dis < min_dis){
                min_dis = dis;
                min_i = i;
            }
        }

        I4 inc = param.target_inc;
        I4 target_i = (min_i+inc)>=line_size?line_size-1:(min_i+inc);
        Pose_6d target = refer_line[target_i];
        R8 Pv = param.Pv;
        R8 Iv = param.Iv;
        R8 Dv = param.Dv;
        R8 Pw = param.Pw;
        R8 Iw = param.Iw;
        R8 Dw = param.Dw;
        dis = (target.x - now_pose.x)*(target.x - now_pose.x) +
              (target.y - now_pose.y)*(target.y - now_pose.y) ;

        R8 yaw = get_angle(target.y,target.x,now_pose.y,now_pose.x);
        R8 yaw_d = yaw_sub(yaw,now_pose.yaw);

        D[2] = D[1];
        D[1] = D[0];
        D[0] = dis;
        Y[2] = Y[1];
        Y[1] = Y[0];
        Y[0] = yaw_d;

        v = Pv*(D[0]) + Iv*(D[0]+D[1]+D[2]) + Dv*(D[0]-D[2]);
        w = Pw*(Y[0]) + Iw*(Y[0]+Y[1]+Y[2]) + Dw*(Y[0]-Y[2]);
        v = v>param.max_v?param.max_v:v;
        v = v<-param.max_v?-param.max_v:v;
        w = w>param.max_w?param.max_w:w;
        w = w<-param.max_w?-param.max_w:w;
        now_i = min_i;
        if(min_i >= line_size-5){
            path_OK = true;
        }
    }

    void Controller::PID_ACM_Model(Pose_6d &now_pose,std::vector<Pose_6d> &refer_line,Control_Param &param){
        //1.find min index
        I4 line_size = refer_line.size();
        I4 min_i = 0;
        R8 min_dis = 999999999;
        R8 dis;
        I4 start_index = (now_i-param.min_index)<0?0:(now_i-param.min_index);
        I4 end_index = (now_i+param.max_index)>=line_size?line_size-1:(now_i+param.max_index);
        for(U4 i = start_index;i<=end_index;i++){
            dis = (refer_line[i].x - now_pose.x)*(refer_line[i].x - now_pose.x) +
                  (refer_line[i].y - now_pose.y)*(refer_line[i].y - now_pose.y) ;
            if(dis < min_dis){
                min_dis = dis;
                min_i = i;
            }
        }
        if(min_dis == 999999999){
            min_i = now_i + 1;
        }

        //2.cal frenet lat and lon
        //2.1 trans to frenet
        Pose_6d frenet_orgin;
        R8 v_dis=0; //vertical dis
        R8 ds;
        U4 sub_i,low_i,high_i;
        if(min_i <= 1 || min_dis == 999999999){
            v_dis = 0;
        } else {
            R8 up_dis,down_dis;
            R8 dl,ul,l;
            R8 cosA;
            R8 sinA;
            up_dis = (now_pose.x - refer_line[min_i+1].x)*(now_pose.x - refer_line[min_i+1].x) +
                     (now_pose.y - refer_line[min_i+1].y)*(now_pose.y - refer_line[min_i+1].y) ;
            down_dis = (now_pose.x - refer_line[min_i-1].x)*(now_pose.x - refer_line[min_i-1].x) +
                       (now_pose.y - refer_line[min_i-1].y)*(now_pose.y - refer_line[min_i-1].y) ;
            sub_i  = up_dis<down_dis?(min_i+1):(min_i-1);
            low_i  = sub_i<min_i?sub_i:min_i;
            high_i = sub_i<min_i?min_i:sub_i;
            ul = up_dis<down_dis?up_dis:min_dis;
            dl = up_dis<down_dis?min_dis:down_dis;
            l  = (refer_line[low_i].x - refer_line[high_i].x)*(refer_line[low_i].x - refer_line[high_i].x) +
                 (refer_line[low_i].y - refer_line[high_i].y)*(refer_line[low_i].y - refer_line[high_i].y) ;
            ul = sqrt(ul);
            dl = sqrt(dl);
            l  = sqrt(l);
            cosA = (l*l + dl*dl - ul*ul)/(2*l*dl);
            sinA = sqrt(1-cosA*cosA);
            v_dis = dl*sinA;
            ds = dl*cosA;
            frenet_orgin.yaw = refer_line[low_i].yaw;
            frenet_orgin.x = refer_line[low_i].x + ds*cos(frenet_orgin.yaw);
            frenet_orgin.y = refer_line[low_i].y + ds*sin(frenet_orgin.yaw);

            R8 to_yaw = get_angle(refer_line[low_i].y,refer_line[low_i].x,now_pose.y,now_pose.x);
            to_yaw = yaw_sub(to_yaw,refer_line[low_i].yaw);
            if( (to_yaw>=0 && to_yaw<TT) ){
                v_dis = -v_dis;
            }
        }

        //2.2 count loss
        R8 lat_loss = -v_dis;
        I4 end_i = (min_i+param.review_num);
        end_i = end_i>=line_size?line_size-1:end_i;
        R8 yaw_loss = 0;
        R8 dis_loss = 0;
        for(I4 i = min_i + 1;i<=end_i;i++){
            yaw_loss += yaw_sub(refer_line[i].yaw,now_pose.yaw);
            dis_loss += refer_line[i].v*param.dt;
        }
        R8 angle_loss = param.ag*lat_loss + param.bg*yaw_loss;
        D[2] = D[1];
        D[1] = D[0];
        D[0] = dis_loss;
        Y[2] = Y[1];
        Y[1] = Y[0];
        Y[0] = angle_loss;

        //3.update PID
        R8 P_lon = param.P_lon;
        R8 I_lon = param.I_lon;
        R8 D_lon = param.D_lon;
        R8 P_lat = param.P_lat;
        R8 I_lat = param.I_lat;
        R8 D_lat = param.D_lat;
        a = P_lon*(D[0]) + I_lon*(D[0]+D[1]+D[2]) + D_lon*(D[0]-D[2]);
        steer = P_lat*(Y[0]) + I_lat*(Y[0]+Y[1]+Y[2]) + D_lat*(Y[0]-Y[2]);
        a = a>param.max_a?param.max_a:a;
        a = a<-param.max_a?-param.max_a:a;
        steer = steer>param.max_steer?param.max_steer:steer;
        steer = steer<-param.max_steer?-param.max_steer:steer;
        now_pose.v = now_pose.v>param.max_v?param.max_v:now_pose.v;
        now_pose.v = now_pose.v<-param.max_v?-param.max_v:now_pose.v;
        now_i = min_i;
        if(min_i >= line_size-5){
            path_OK = true;
        }
    }

    void Controller::MPC_ACM_Model(Pose_6d &now_pose,std::vector<Pose_6d> &refer_line,Control_Param &param){
        ////,you should git clone udaicty MPC code:MPC.cpp, MPC.h,ipopt
          //1.find min index
//        I4 line_size = refer_line.size();
//        I4 min_i = 0;
//        R8 min_dis = 999999999;
//        R8 dis;
//        I4 start_index = (now_i-param.min_index)<0?0:(now_i-param.min_index);
//        I4 end_index = (now_i+param.max_index)>=line_size?line_size-1:(now_i+param.max_index);
//        for(U4 i = start_index;i<=end_index;i++){
//            dis = (refer_line[i].x - now_pose.x)*(refer_line[i].x - now_pose.x) +
//                  (refer_line[i].y - now_pose.y)*(refer_line[i].y - now_pose.y) ;
//            if(dis < min_dis){
//                min_dis = dis;
//                min_i = i;
//            }
//        }
//        if(min_dis == 999999999){
//            min_i = now_i + 2;
//        }

        //2.count frenet;
//        U4 N = 20;//if you wanyt to change N ,change it at MPC.cpp line 8 also
        //for change wheel base you should change MPC.cpp line 24

        //2.1 achieve 1
//        Eigen::VectorXd ptsx_car(N);
//        Eigen::VectorXd ptsy_car(N);
//        R8 yaw = -now_pose.yaw;
//        for(I4 i = min_i+1;i<=min_i+N;i++){
//            R8 x = refer_line[i].x - now_pose.x;
//            R8 y = refer_line[i].y - now_pose.y;
//            ptsx_car(i-min_i-1) = x * cos(yaw) - y * sin(yaw);
//            ptsy_car(i-min_i-1) = x * sin(yaw) + y * cos(yaw);
//        }
//        auto coeffs = polyfit(ptsx_car, ptsy_car, 3);

        //2.2 achieve 2
//        Eigen::VectorXd coeffs(4);
//        coeffs(1) = 0;
//        Pose_6d frenet_orgin;
//        R8 v_dis=0; //vertical dis
//        R8 ds;
//        U4 sub_i,low_i,high_i;
//        if(min_i <= 1 || min_dis == 999999999){
//            v_dis = 0;
//        } else {
//            R8 up_dis,down_dis;
//            R8 dl,ul,l;
//            R8 cosA;
//            R8 sinA;
//            up_dis = (now_pose.x - refer_line[min_i+1].x)*(now_pose.x - refer_line[min_i+1].x) +
//                     (now_pose.y - refer_line[min_i+1].y)*(now_pose.y - refer_line[min_i+1].y) ;
//            down_dis = (now_pose.x - refer_line[min_i-1].x)*(now_pose.x - refer_line[min_i-1].x) +
//                       (now_pose.y - refer_line[min_i-1].y)*(now_pose.y - refer_line[min_i-1].y) ;
//            sub_i  = up_dis<down_dis?(min_i+1):(min_i-1);
//            low_i  = sub_i<min_i?sub_i:min_i;
//            high_i = sub_i<min_i?min_i:sub_i;
//            ul = up_dis<down_dis?up_dis:min_dis;
//            dl = up_dis<down_dis?min_dis:down_dis;
//            l  = (refer_line[low_i].x - refer_line[high_i].x)*(refer_line[low_i].x - refer_line[high_i].x) +
//                 (refer_line[low_i].y - refer_line[high_i].y)*(refer_line[low_i].y - refer_line[high_i].y) ;
//            ul = sqrt(ul);
//            dl = sqrt(dl);
//            l  = sqrt(l);
//            cosA = (l*l + dl*dl - ul*ul)/(2*l*dl);
//            sinA = sqrt(1-cosA*cosA);
//            v_dis = dl*sinA;
//            ds = dl*cosA;
//            frenet_orgin.yaw = refer_line[low_i].yaw;
//            frenet_orgin.x = refer_line[low_i].x + ds*cos(frenet_orgin.yaw);
//            frenet_orgin.y = refer_line[low_i].y + ds*sin(frenet_orgin.yaw);
//
//            R8 to_yaw = get_angle(refer_line[low_i].y,refer_line[low_i].x,now_pose.y,now_pose.x);
//            to_yaw = yaw_sub(to_yaw,refer_line[low_i].yaw);
//            if( (to_yaw>=0 && to_yaw<TT) ){
//                v_dis = -v_dis;
//            }
//            coeffs(1) = tan(to_yaw);
//        }

        //3.count bias
//        coeffs(0) = v_dis;
//        coeffs(2) = 0;
//        coeffs(3) = 0;
//        printf("cf:%f,%f,%f,%f\n",coeffs[0],coeffs[1],coeffs[2],coeffs[3]);
//        R8 cte = coeffs(0);
//        R8 epsi = atan(coeffs[1]);
//        R8 dt = param.dt;
//        R8 pred_px = 0.0 + now_pose.v*dt; // Since psi is zero, cos(0) = 1, can leave out
//        const R8 pred_py = 0.0; // Since sin(0) = 0, y stays as 0 (y + v * 0 * dt)
//        R8 pred_psi = 0.0 + now_pose.v*(steer)/wheel_base*dt;
//        R8 pred_v = now_pose.v + a*dt;
//        R8 pred_cte = cte + now_pose.v*sin(epsi)*dt;
//        R8 pred_epsi = epsi + now_pose.v*(steer)/wheel_base*dt;
//        Eigen::VectorXd state(6);
//        state << pred_px, pred_py, -pred_psi, pred_v, -pred_cte, -pred_epsi;
//
//        std::vector<R8> v_list;
//        for(I4 i = min_i+1;i<=min_i+N;i++){
//            v_list.push_back(refer_line[i].v);
//        }
//
//        //4.solve
//        MPC mpc;
//        vector<R8> vars = mpc.Solve(state,coeffs,v_list,N,0);
//        steer = vars[0];
//        a = vars[1];
//        now_i = min_i;
//        printf("pe:%f,%f:op:%f,%f\n",cte,pred_cte,epsi,pred_epsi);
//        printf("loss:%f,%f\n",cte,coeffs(1));
//        printf("min_i:%d,%f,%f\n",min_i,a,steer);
    }
}