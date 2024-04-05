#include "sapf.h"
#include<math.h>

namespace Local_Planning
{

void SAPF::init(ros::NodeHandle& nh)
{
    has_local_map_ = false;

    nh.param("sapf/inflate_distance", inflate_distance, 0.20);  // 障碍物膨胀半径
    nh.param("sapf/sensor_max_range", sensor_max_range, 2.5);  // 感知障碍物距离
    nh.param("sapf/k_push", k_push, 0.8);                         // 推力增益
    nh.param("sapf/k_att", k_att, 0.4);                                  // 引力增益
    nh.param("sapf/min_dist", min_dist, 0.2);                            // 最小壁障距离
    nh.param("sapf/max_att_dist", max_att_dist, 5.0);             // 最大吸引距离
    nh.param("sapf/ground_height", ground_height, 0.1);  // 地面高度
    nh.param("sapf/ground_safe_height", ground_safe_height, 0.2);  // 地面安全距离
    nh.param("sapf/safe_distance", safe_distance, 0.15); // 安全停止距离
    // root1 adds sapf parameters
    nh.param("sapf/dsafe", dsafe, 0.5);
    nh.param("sapf/dvort", dvort, 0.65);
    nh.param("sapf/alpha_th", alpha_th, 5*M_PI/180);


    // TRUE代表2D平面规划及搜索,FALSE代表3D 
    nh.param("local_planner/is_2D", is_2D, true); 
}

void SAPF::set_local_map(sensor_msgs::PointCloud2ConstPtr &local_map_ptr)
{
    local_map_ptr_ = local_map_ptr;
    // root1 comments on 141123
    // ros::Time begin_load_point_cloud = ros::Time::now();

    pcl::fromROSMsg(*local_map_ptr, latest_local_pcl_);

    has_local_map_=true;
}

void SAPF::set_local_map_pcl(pcl::PointCloud<pcl::PointXYZ>::Ptr &pcl_ptr)
{
    latest_local_pcl_ = *pcl_ptr;
    has_local_map_=true;
}

void SAPF::set_odom(nav_msgs::Odometry cur_odom)
{
    cur_odom_ = cur_odom;
    has_odom_=true;
}

// root1 adds desired_yaw in apf.cpp, apf.h, vfh.cpp, vfh.h
int SAPF::compute_force(Eigen::Vector3d &goal, Eigen::Vector3d &desired_vel, float &desired_yaw)
{
    // 0 for not init; 1 for safe; 2 for dangerous
    int local_planner_state=0;
    int safe_cnt=0;

    if(!has_local_map_|| !has_odom_)
        return 0;

    if ((int)latest_local_pcl_.points.size() == 0) 
        return 0;

    if (isnan(goal(0)) || isnan(goal(1)) || isnan(goal(2)))
        return 0;

    //　当前位置
    Eigen::Vector3d current_pos;
    current_pos[0] = cur_odom_.pose.pose.position.x;
    current_pos[1] = cur_odom_.pose.pose.position.y;
    current_pos[2] = cur_odom_.pose.pose.position.z;
    // root1 adds on 121123
    // 获取当前偏航角
    // convert quaternion to rpy
    tf::Quaternion current_quat;
    tf::quaternionMsgToTF(cur_odom_.pose.pose.orientation, current_quat);
    double current_r, current_p, current_y;
    tf::Matrix3x3(current_quat).getRPY(current_r, current_p, current_y);

    // root1 comments on 151123
    // ros::Time begin_collision = ros::Time::now();

    // 引力
    Eigen::Vector3d uav2goal = goal - current_pos;
    // 不考虑高度影响
    uav2goal(2) = 0.0;
    double dist_att = uav2goal.norm(); 
    if(dist_att > max_att_dist)
    {
        uav2goal = max_att_dist * uav2goal/dist_att ;  
    }
    //　计算吸引力
    attractive_force = k_att * uav2goal;  // root1: checked

    // 排斥力
    double uav_height = cur_odom_.pose.pose.position.z;
    push_force = Eigen::Vector3d(0.0, 0.0, 0.0);  // root1: 因为障碍物的坐标是在无人机的坐标系下表示的

    Eigen::Vector3d p3d;  // root1: stores the position of the obstacle
    vector<Eigen::Vector3d> obstacles;
    
    //　根据局部点云计算排斥力
    for (size_t i = 0; i < latest_local_pcl_.points.size(); ++i)  // 障碍物个数
    {
        p3d(0) = latest_local_pcl_.points[i].x;
        p3d(1) = latest_local_pcl_.points[i].y;
        p3d(2) = latest_local_pcl_.points[i].z;

        // 实验证明不能不用local
        Eigen::Vector3d current_pos_local(0.0, 0.0, 0.0);  // root1: used for what? why not get it from odom?
        
        //　低于地面高度，则不考虑该点的排斥力
        double point_height_global = uav_height+p3d(2);
        if(fabs(point_height_global)<ground_height)
            continue;

        //　超出最大感知距离，则不考虑该点的排斥力
        double dist_push = (current_pos_local - p3d).norm();
        if(dist_push > sensor_max_range || isnan(dist_push))
            continue;

        //　考虑膨胀距离
        dist_push = dist_push - inflate_distance;

        // 如果当前的观测点中，包含小于安全停止距离的点，进行计数
        if(dist_push < safe_distance)
        {
            safe_cnt++;
        }
        
        //　小于最小距离时，则增大该距离，从而增大排斥力
        if(dist_push < min_dist)
        {
            dist_push = min_dist /1.5;
        }

        obstacles.push_back(p3d);
        double push_gain; // = k_push * (1/dist_push - 1/sensor_max_range)* 1.0/(dist_push * dist_push);

        // root1 moves below
        /*if(dist_att<1.0)
        {
            push_gain *= dist_att;  // to gaurantee to reach the goal.
        }*/

        // root1 adds on 121123
        // Compute R
        // U_obst(q) = R(gamma) * U_rep(q)
        // alpha is the angle between the front of the drone and the obstacle
        double alpha = current_y - std::atan2(p3d(1) - current_pos[1], p3d(0) - current_pos[0]);
        alpha = std::atan2(sin(alpha), cos(alpha));
        if(dist_push <= sensor_max_range && fabs(alpha) < (150*M_PI/180))
            push_gain = k_push * (1/dist_push - 1/sensor_max_range)* 1.0/(dist_push * dist_push);  // root1: 这边正负号有点问题? 因为排斥力是负的?

         // 这两行本来就有
         if(dist_att<1.0)
            push_gain *= dist_att;  // to gaurantee to reach the goal.

        double drel_Oi;
        if(dist_push <= dsafe)
            drel_Oi = 0;
        else if(dist_push >= 2*dvort-dsafe)
            drel_Oi = 1;
        else
            drel_Oi = (dist_push - dsafe) / (2*(dvort - dsafe));

        double D_alpha;
        if(alpha*180/M_PI <=alpha_th)
            D_alpha = 1;
        else
            D_alpha = -1;

        double gamma;
        if(drel_Oi <= 0.5)
            gamma = M_PI * D_alpha * drel_Oi;
        else
            gamma = M_PI * D_alpha * (1 - drel_Oi);

        Eigen::Matrix<double,2,2> R_sapf;
        R_sapf(0, 0) = cos(gamma);
        R_sapf(0, 1) = -sin(gamma);
        R_sapf(1, 0) = sin(gamma);
        R_sapf(1, 1) = cos(gamma);
        // R_sapf << cos(gamma), -sin(gamma),
                             // sin(gamma), cos(gamma);

        // 这行本来就有
        push_force += push_gain * (current_pos_local - p3d)/dist_push;

        // root1 adds on 121123
        push_force_2d[0] = push_force[0];
        push_force_2d[1] = push_force[1];

        push_force_2d = push_force_2d + (R_sapf * push_force_2d);
    }

    // root1 adds on 121123
    push_force[0] = push_force_2d[0];
    push_force[1] = push_force_2d[1];

    //　平均排斥力
    if(obstacles.size() != 0)
    {
        push_force=push_force/obstacles.size();  // root1: 为什么要计算平均值而不是总和？注释掉看看效果
    }

    // root1: 可以直接在全局坐标系计算吗？
    // root1: 将平均排斥力转换到全局坐标系 确保得到的力是相对于全局坐标系的 因为吸引力是在全局坐标系中进行的
    Eigen::Quaterniond cur_rotation_local_to_global(cur_odom_.pose.pose.orientation.w, 
                                                    cur_odom_.pose.pose.orientation.x,  
                                                    cur_odom_.pose.pose.orientation.y,  
                                                    cur_odom_.pose.pose.orientation.z);  // root1: ? why use rotation matrix？


    Eigen::Matrix<double,3,3> rotation_mat_local_to_global = cur_rotation_local_to_global.toRotationMatrix();

    push_force = rotation_mat_local_to_global * push_force; 

    // 合力
    // root1 modifies on 121123
    desired_vel = push_force + attractive_force;
    // root1 comments on 151123, put it in the local_planning.cpp
    // desired_yaw = std::atan2(desired_vel[1], desired_vel[0]);

    if(is_2D)
    {
        desired_vel[2] = 0.0;
    }

    // 如果不安全的点超出，
    if(safe_cnt>10)
    {
        local_planner_state = 2;  //成功规划，但是飞机不安全
    }else
    {
        local_planner_state =1;  //成功规划， 安全
    }

    static int exec_num=0;
    exec_num++;

    // 此处改为根据循环时间计算的数值
    // root1 comments on 151123
    /*
    if(exec_num == 50)
    {
        printf("SAPF calculate take %f [s].\n",   (ros::Time::now()-begin_collision).toSec());
        exec_num=0;
    }*/

    return local_planner_state;
}



}
