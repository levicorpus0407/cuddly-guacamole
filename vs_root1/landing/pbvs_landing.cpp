/**
 * \file pbvs_landing.cpp
 * \brief  visual servoing for uav landing
 * \author Zixuan XU                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                or  XU Zixuan
 * \version 0.1
 * \date 20/11/2022
 * 
 * \param[in] 
 * 
 * Subscribes to: <BR>
 *    ° 
 * 
 * Publishes to: <BR>
 *    ° 
 *
 * Description
 * s: get from the coordinates that is published by the landing pad
 * s*: center point set to zero
 * vt and u should be expressed in the body frame
 * We do not need enu frame in visual servoing
 * 
 * Current issues
 * 运行节点后, 无人机上升飞走了 solved
 * 如何判断是哪个标识码被检测到了? solved
 * 动平台降落失败 solved
 * 速度控制指令发送到? px4_sender.cpp
 * 控制指令直接改到body frame可以吗 目前看来没问题
  */


//Cpp
#include <sstream>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

//ROS
// #include "ros/ros.h"

// Visp
#include <visp/vpFeatureDepth.h>
#include <visp/vpFeatureTranslation.h>
#include <visp/vpFeatureThetaU.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpFeaturePointPolar.h>
#include <visp/vpMatrix.h>
#include <visp/vpColVector.h>
#include <visp/vpServo.h>
#include <visp3/vs/vpServo.h>
#include <visp/vpFeatureBuilder.h>
#include <visp3/core/vpVelocityTwistMatrix.h>
#include <visp3/vs/vpAdaptiveGain.h>
#include <visp/vpExponentialMap.h>
// root1 adds on 190623
#include <visp3/visual_features/vpFeatureThetaU.h>
#include <visp3/visual_features/vpFeatureTranslation.h>
#include <visp3/core/vpRotationMatrix.h>

#include <log2plot/logger.h>
#include <log2plot/dir_tools.h>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>

#include <cmath>
#include <vector>

#include <prometheus_msgs/DetectionInfo.h>

// Include here the ".h" files corresponding to the topic type you use.
#include "mission_utils.h"
#include "message_utils.h"

using namespace std;
using namespace Eigen;

#define NODE_NAME "pbvs_landing"
#define ROOT1_PATH "/home/zixuanxu/root1/simulation/"

// You may have a number of globals here.
// prometheus
bool hold_mode; // 悬停模式，用于测试检测精度
bool sim_mode;  // 选择Gazebo仿真模式 或 真实实验模式
bool use_pad_height;  // 是否使用降落板绝对高度
float pad_height;
string message;
std_msgs::Bool vision_switch;
geometry_msgs::PoseStamped mission_cmd;
float start_point[3];    // 起始降落位置
float camera_offset[3];
// root1 adds on 270623
bool start_move_flag = false;
double start_move_distance = 0.0;
bool force_landing_flag = false;

// root1 adds
// std_msgs::Float32MultiArray target_vel;
std_msgs::Float32MultiArray cRtm_msg;
geometry_msgs::Twist target_twist, target_twist_check;
// vector<float> cRtm;
// std_msgs::Float32 vs_error_x, vs_error_y, vs_error_z;
// float target_vel_xy[2];  // 目标移动速度 enu坐标系 单位：m/s (vs需要target velocity在body frame下表示, 怎么转换?)
std_msgs::Bool flag_start;
//---------------------------------------Drone---------------------------------------------
prometheus_msgs::DroneState _DroneState;    // 无人机状态
Eigen::Matrix3f R_Body_to_ENU;   // 无人机机体系至惯性系转换矩阵 body frame to ENU frame
//---------------------------------------Vision---------------------------------------------
nav_msgs::Odometry GroundTruth;  // 降落板真实位置（仿真中由Gazebo插件提供）
Detection_result landpad_det;   // 检测结果 视觉检测原始信息，返回的结果为相机坐标系
// Detection_result corners_det;  // root1 adds 储存四个角点在相机坐标系下的坐标

//---------------------------------------Track---------------------------------------------
// root1 visp parameters
double lambda; // visual servo gain to be tuned
double lambda_0, lambda_inf;
vpTranslationVector ctb, etb, bte, btc, ctm, mttar, eab;
vpRotationMatrix cRb, eRb, bRe, bRc, cRm, mRtar, cRm1;
vpColVector u(4), u1(4), vt(4), bvt(6), ev(6), e(6), mv(6), cv1(6), tarv(6), bv_check(6);
vpColVector XYZ_corners(12);
vpMatrix J(6, 3), J1(6, 4);
double k_vs[4];  // vs parameters for vx, vy, vz, yaw_rate
double hd;  // desired height
double kvt_vs[4];  // parameter to adjust vt for LAND mode
int iter = 0;
int vs_id = 0;
bool vff_received = false;
bool ground_truth_received = false;
bool transformation_received = false;
bool transformation_received1 = false;
double current_t = 0.0; // compute time during the tracking process
double sample_time = 1;
// vectors
// ctm: translational vector from the target frame to the camera frame
// ctb: translational vector from the body frame to the camera frame
// u: velocity control command, vx, vy, vz, yaw_rate
// vt: the velocity of the vehicle expressed in the body frame
// etb: translational vector from the body frame to the ENU frame, bte: translational vector from the ENU frame to the body frame
// bvt: twist expressed in the body frame, ev: velocity twist expressed in the ENU frame
// e: vs error

// matrices
// cRb: rotation matrix from the body frame to the camera frame
// cWb: velocity twist matrix from the body frame to the camera frame
// J: jacobian matrix
// eRb: rotation matrix from the body frame to the ENU frame, bRe: rotation matrix from the ENU frame to the body frame
// bWe: velocity twist matrix from the ENU frame to the body frame

// land pad length
double landpad_det_len = 0.6;
double len_b = landpad_det_len*0.666667f;  // 0.4
double len_m =  landpad_det_len*0.133334f;  // 0.08
double len_s = landpad_det_len*0.066667f;  // 0.04
double len_id = (len_b + len_m)/2;
 
// 五种状态机 (四种?)
enum EXEC_STATE
{
    WAITING_RESULT,
    TRACKING,
    LOST,
    LANDING,
};
EXEC_STATE exec_state;

float distance_to_pad;
float arm_height_to_ground;
float arm_distance_to_pad;
//---------------------------------------Output---------------------------------------------
prometheus_msgs::ControlCommand Command_Now;   //发送给控制模块 [px4_sender.cpp]的命令
// 函数声明
void printf_param();    //打印各项参数以供检查
void printf_result();
// root1 visp skew matrix
// vpMatrix skew(vpColVector ctb);

// Callback functions
// prometheus           
void landpad_det_cb(const prometheus_msgs::DetectionInfo::ConstPtr &msg)  // 降落版检测
{
    landpad_det.object_name = "landpad";
    landpad_det.Detection_info = *msg;
    transformation_received = true;
    vs_id = landpad_det.Detection_info.det_id;
    // the target position published by the landpad detection is in the camera frame
    // 识别算法发布的目标位置位于相机坐标系（从相机往前看，物体在相机右方x为正，下方y为正，前方z为正）camera points down
    // 现在要将目标位置转换到机体坐标系( but visual servoing requires the target pose (features) to be  in the camera frame? No? not sure)
    // 相机安装误差 在mission_utils.h中设置(? we set this in launch file) camera offset 是相对于机体系的
    // pos_body_frame为target在body frame下的pose bp(camera frame-->body frame)
    landpad_det.pos_body_frame[0] = - landpad_det.Detection_info.position[1] + camera_offset[0]; 
    landpad_det.pos_body_frame[1] = - landpad_det.Detection_info.position[0] + camera_offset[1];
    landpad_det.pos_body_frame[2] = - landpad_det.Detection_info.position[2] + camera_offset[2];  // z轴反向并加上相机偏置

    // wpb = wRb * bp + wtb (body frame-->world frame)
    // 机体系 -> 机体惯性系 (原点在机体的惯性系) (对无人机姿态进行解耦)
    landpad_det.pos_body_enu_frame = R_Body_to_ENU * landpad_det.pos_body_frame; 

    if(use_pad_height)  // pad_height: 降落板绝对高度
    {
        //若已知降落板高度，则无需使用深度信息。
        // 计算出来的是无人机距离降落板的高度差
        landpad_det.pos_body_enu_frame[2] = pad_height - _DroneState.position[2];  // h 如果高度定位不准 这个高度差也不准
    }

    // root1 test
    // cout << "z_pad in the pos_body_enu_frame is " << landpad_det.pos_body_enu_frame[2] << endl;
    // cout << "the height of drone is" <<  _DroneState.position[2] << endl;

    // 机体惯性系 -> 惯性系  In PID control, the velocity command is expressed in the ENU frame (use this to check the computer vision part)
    landpad_det.pos_enu_frame[0] = _DroneState.position[0] + landpad_det.pos_body_enu_frame[0];
    landpad_det.pos_enu_frame[1] = _DroneState.position[1] + landpad_det.pos_body_enu_frame[1];
    landpad_det.pos_enu_frame[2] = _DroneState.position[2] + landpad_det.pos_body_enu_frame[2];  // =pad_height 降落板绝对高度
    
    // store XY of corners
    // for(int i=0; i<12; i++)
        // XYZ_corners[i] = landpad_det.Detection_info.corner_position[i];

    // 此降落方案不考虑偏航角 （高级版可提供）delete?
    landpad_det.att_enu_frame[2] = 0.0;

    double flag_detection = landpad_det.Detection_info.detected;
    if(flag_detection)
        ROS_INFO("flag = true");
    else
        ROS_INFO("flag = false");

    if(landpad_det.Detection_info.detected)
    {
        landpad_det.num_regain++;
        landpad_det.num_lost = 0;
    }else
    {
        landpad_det.num_regain = 0;
        landpad_det.num_lost++;
    }

    // 当连续一段时间无法检测到目标时，认定目标丢失
    if(landpad_det.num_lost > VISION_THRES)
    {
        landpad_det.is_detected = false;
        ROS_INFO_STREAM_ONCE("DETECTED FALSE");
    }

    // 当连续一段时间检测到目标时，认定目标得到
    if(landpad_det.num_regain > VISION_THRES)
    {
        landpad_det.is_detected = true;
        ROS_INFO_STREAM_ONCE("DETECTED TRUE");
    }

}

void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg)  // 无人机状态
{
    _DroneState = *msg;

    // 机体系到惯性系的旋转矩阵(3, 3)
    R_Body_to_ENU = get_rotation_matrix(_DroneState.attitude[0], _DroneState.attitude[1], _DroneState.attitude[2]);

    // root1 adds: store eRb
    for(int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++)
            eRb[i][j] = R_Body_to_ENU(i, j);
    }

    etb[0] = _DroneState.position[0];
    etb[1] = _DroneState.position[1];
    etb[2] = _DroneState.position[2];

    // eab[0] = _DroneState.attitude[0];
    // eab[1] = _DroneState.attitude[1];
    // eab[2] = _DroneState.attitude[2];
      
}

void groundtruth_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    GroundTruth = *msg;
    // target_twist_check = GroundTruth.twist.twist;
    ground_truth_received = true;
}

void switch_cb(const std_msgs::Bool::ConstPtr& msg)  // 降落版程序开关，多任务时启动
{
    flag_start = *msg;
}

void mission_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)  //用于中断任务，直接降落
{
    mission_cmd = *msg;
}

// root1 adds
/*
void targetVel_cb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    target_vel = *msg;
    target_vel_xy[0] = target_vel.data.at(0);
    target_vel_xy[1] = target_vel.data.at(1);
}*/

void targetTwist_cb(const geometry_msgs::Twist::ConstPtr& msg)
{
    target_twist = *msg;
    vff_received = true;
}

void transformation_cb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    cRtm_msg = *msg;
    // cRtm = cRtm_msg.data;
    transformation_received1 = true;
}

int main (int argc, char** argv)
{

	//ROS Initialization
    ros::init(argc, argv, "pbvs_landing");

    // Define your node handles: YOU NEED AT LEAST ONE !
    ros::NodeHandle nh("~");  // local

    ros::Rate rate(20.0);

    // Declare your node's subscriptions and service clients
    // from prometheus
    //【订阅】降落板与无人机的相对位置及相对偏航角  单位：米   单位：弧度
    //  方向定义： 识别算法发布的目标位置位于相机坐标系（从相机往前看，物体在相机右方x为正，下方y为正，前方z为正）
    //  标志位：   detected 用作标志位 true代表识别到目标 false代表丢失目标
    ros::Subscriber landpad_det_sub = nh.subscribe<prometheus_msgs::DetectionInfo>("/prometheus/object_detection/landpad_det", 10, landpad_det_cb);

    //【订阅】无人机状态
    ros::Subscriber drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 10, drone_state_cb);

    //【订阅】地面真值，此信息仅做比较使用 不强制要求提供
    // ros::Subscriber groundtruth_sub = nh.subscribe<nav_msgs::Odometry>("/ground_truth/landing_pad", 10, groundtruth_cb);
    ros::Subscriber groundtruth_sub = nh.subscribe<nav_msgs::Odometry>("/ground_truth/diff", 10, groundtruth_cb);

    //【订阅】用于中断任务，直接降落
    ros::Subscriber mission_sub = nh.subscribe<geometry_msgs::PoseStamped>("/prometheus/mission/cmd", 10, mission_cb);

    //【订阅】降落程序开关，默认情况下不启用，用于多任务情况
    ros::Subscriber switch_sub = nh.subscribe<std_msgs::Bool>("/prometheus/switch/landing", 10, switch_cb);

    //  root1 adds
    // 【订阅】小车速度
    // ros::Subscriber vel_sub = nh.subscribe<std_msgs::Float32MultiArray>("/root1/target_vel", 10, targetVel_cb);
    // ros::Subscriber targetTwist_sub = nh.subscribe<geometry_msgs::TwistStamped>("/cmd_vel_enu", 10, targetTwist_cb);
    ros::Subscriber targetTwist_sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel_diff", 10, targetTwist_cb);
    // ros::Subscriber targetTwist_sub = nh.subscribe<geometry_msgs::Twist>("/vs/object/cmd_vel", 10, targetTwist_cb);
    ros::Subscriber transformation_sub = nh.subscribe<std_msgs::Float32MultiArray>("/vs/transformation", 10, transformation_cb);

    // Declare you publishers and service servers
    // from prometheus
    // 【发布】 视觉模块开关量
    ros::Publisher vision_switch_pub = nh.advertise<std_msgs::Bool>("/prometheus/switch/ellipse_det", 10);

    //【发布】发送给控制模块 [px4_pos_controller.cpp]的命令
    ros::Publisher command_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);

    // 【发布】用于地面站显示的提示消息
    ros::Publisher message_pub = nh.advertise<prometheus_msgs::Message>("/prometheus/message/main", 10); 

    // 【发布】 vs信息
    // ros::Publisher vsError_x_pub = nh.advertise<std_msgs::Float32>("/vs/error/x", 10);
    // ros::Publisher vsError_y_pub = nh.advertise<std_msgs::Float32>("/vs/error/y", 10);
    // ros::Publisher vsError_z_pub = nh.advertise<std_msgs::Float32>("/vs/error/z", 10);
    ros::Publisher vsError_pub = nh.advertise<std_msgs::Float32MultiArray>("/vs/error", 10);

    // root1 adds on 271023
    // 【发布】 降落板的估计位姿在世界坐标系下的坐标和真值的坐标
    ros::Publisher cvDection_pub = nh.advertise<std_msgs::Float32MultiArray>("/cv/detection", 10);

    // Read the node parameters if any
    // prometheus
    // 参数读取
    //强制上锁高度
    nh.param<float>("arm_height_to_ground", arm_height_to_ground, 0.4);
    //强制上锁距离
    nh.param<float>("arm_distance_to_pad", arm_distance_to_pad, 0.3);
    // 悬停模式 - 仅用于观察检测结果
    nh.param<bool>("hold_mode", hold_mode, false);
    // 仿真模式 - 区别在于是否自动切换offboard模式
    nh.param<bool>("sim_mode", sim_mode, true);
    // 是否使用降落板绝对高度
    nh.param<bool>("use_pad_height", use_pad_height, false);
    nh.param<float>("pad_height", pad_height, 0.01);

    // root1 adds vs parameters

    nh.param<double>("hd", hd, 0.5);

    nh.param<double>("kx_vs", k_vs[0], 0.1);
    nh.param<double>("ky_vs", k_vs[1], 0.1);
    nh.param<double>("kz_vs", k_vs[2], 0.1);
    nh.param<double>("kyaw_vs", k_vs[3], 0.1);

    // 初始起飞点
    nh.param<float>("start_point_x", start_point[0], 0.0);
    nh.param<float>("start_point_y", start_point[1], 0.0);
    nh.param<float>("start_point_z", start_point[2], 1.0);

    // 相机安装偏移,规定为:相机在机体系(质心原点)的位置  btc
    nh.param<float>("camera_offset_x", camera_offset[0], 0.0);
    nh.param<float>("camera_offset_y", camera_offset[1], 0.0);
    nh.param<float>("camera_offset_z", camera_offset[2], 0.0);

    // 速度控制指令中的lambda系数
    nh.param<double>("lambda", lambda, 1.4);
    nh.param<double>("lambda_0", lambda_0, 2.0);
    nh.param<double>("lambda_inf", lambda_inf, 1.0);

    nh.param<double>("kvt_x", kvt_vs[0], 1.2);
    nh.param<double>("kvt_y", kvt_vs[1], 1.2);
    nh.param<double>("kvt_z", kvt_vs[2], 1.2);
    nh.param<double>("kvt_w", kvt_vs[3], 1.2);

    // root1 adds on 270623
    nh.param<bool>("start_move_flag", start_move_flag, false);
    nh.param<double>("start_move_distance", start_move_distance, 0.0);
    nh.param<bool>("force_landing_flag", force_landing_flag, false);

    //打印现实检查参数
    printf_param();

    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    cout<<setprecision(2);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);


    Command_Now.Command_ID = 1;
    Command_Now.source = NODE_NAME;

    // root1 add plot
    /*log2plot::closePreviousPlots();
    // pose of a camera expressed as world pose in camera frame
    vpHomogeneousMatrix M(0,0,0.1,M_PI/2,0,0);
    vpPoseVector pose;
    // camera velocity in its own frame
    vpColVector vc(6);
    log2plot::Logger logger(ROOT1_PATH);
    // save velocity with Latex legend
    logger.save(vc, "visp_velocity", "[v_x,v_y,v_z,\\omega_x,\\omega_y,\\omega_z]", "Camera velocity");
    logger.showFixedObject({{30,.4},{40,.8},{60,.2},{80,.6}}, log2plot::legendFullyConnected(4), "r");
    // save pose as iteration-based with Latex legend
    logger.save(pose, "visp_pose", "[x,y,z,\\theta_x,\\theta_y,\\theta_z]", "Camera pose");
    // save pose as 3D plot
    // the saved variable is the world pose in camera frame, we want to plot the invert
    logger.save3Dpose(pose, "visp_pose3D", "Camera pose", true);
    logger.setLineType("[C0,C1,C2,C3]");
    // add a camera along the trajectory
    logger.showMovingCamera();
    // add a box
    logger.showFixedBox(-1,-1,-8,3,4,0, "C6");
    // default script path + verbose
    logger.plot(true);*/

    if(sim_mode)
    {
        cout << "It is sim mode." << endl;
        // Waiting for input
        int start_flag = 0;
        while(start_flag == 0)
        {
            cout << ">>>>>>>>>>>>>>>>>>>>>>>>>Autonomous Landing Mission<<<<<<<<<<<<<<<<<<<<<< "<< endl;
            cout << "Please check the parameter and setting, enter 1 to continue,  else for quit: "<<endl;
            cin >> start_flag;
        }

        while(ros::ok() && _DroneState.mode != "OFFBOARD")
        {
            Command_Now.header.stamp = ros::Time::now();
            Command_Now.Mode  = prometheus_msgs::ControlCommand::Idle;
            Command_Now.Command_ID = Command_Now.Command_ID + 1;
            Command_Now.source = NODE_NAME;
            Command_Now.Reference_State.yaw_ref = 999;
            command_pub.publish(Command_Now);   
            cout << "Switch to OFFBOARD and arm ..."<<endl;
            ros::Duration(2.0).sleep();
            ros::spinOnce();
        }
    }else
    {
        while(ros::ok() && _DroneState.mode != "OFFBOARD")
        {
            cout << "Waiting for the offboard mode"<<endl;
            ros::Duration(1.0).sleep();
            ros::spinOnce();
        }
    }

    // 起飞
    cout<<"[autonomous_landing]: "<<"Takeoff to predefined position."<<endl;
    pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, "Takeoff to predefined position.");

    while( _DroneState.position[2] < 0.5)
    {      
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode = prometheus_msgs::ControlCommand::Move;
        Command_Now.Command_ID = Command_Now.Command_ID + 1;
        Command_Now.source = NODE_NAME;
        Command_Now.Reference_State.Move_mode  = prometheus_msgs::PositionReference::XYZ_POS;
        Command_Now.Reference_State.Move_frame = prometheus_msgs::PositionReference::ENU_FRAME;
        Command_Now.Reference_State.position_ref[0] = start_point[0];
        Command_Now.Reference_State.position_ref[1] = start_point[1];
        Command_Now.Reference_State.position_ref[2] = start_point[2];
        Command_Now.Reference_State.yaw_ref = 0.0;
        command_pub.publish(Command_Now);
        cout << "Takeoff ..."<<endl;
        ros::Duration(3.0).sleep();

        ros::spinOnce();
    }

    // 等待
    ros::Duration(3.0).sleep();

    exec_state = EXEC_STATE::WAITING_RESULT;

    while (ros::ok())
    {
        ros::spinOnce();

        std_msgs::Float32MultiArray vs_error, w_det_pose;

         // root1 adds on 271023 for cv detection message in the world frame
        // not sure if it is proper to publish here. maybe it is better to put it here than the tracking case
        // if the landing pad is detected, the message will be published.
        w_det_pose.data.push_back(landpad_det.pos_enu_frame[0]);
        w_det_pose.data.push_back(landpad_det.pos_enu_frame[1]);
        w_det_pose.data.push_back(landpad_det.pos_enu_frame[2]);
        if(ground_truth_received==true){
            w_det_pose.data.push_back(GroundTruth.pose.pose.position.x);
            w_det_pose.data.push_back(GroundTruth.pose.pose.position.y);
            w_det_pose.data.push_back(GroundTruth.pose.pose.position.z);
        }
        cvDection_pub.publish(w_det_pose);

        static int printf_num = 0;
        printf_num++;
        // 此处是为了控制打印频率
        if(printf_num > 20)
        {
            if(exec_state == TRACKING)
            {
                // 正常追踪
                char message_chars[256];
                sprintf(message_chars, "Tracking the Landing Pad, distance_to_the_pad :   %f [m] .", distance_to_pad);
                message = message_chars;
                cout << message <<endl;
                pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, message);
            }

            if(sim_mode)
            {
                printf_result();
            }
            
            printf_num = 0;
        }

        // 接收到中断指令，直接降落
        if(mission_cmd.pose.position.x == 99)
        {
            exec_state = LANDING;
        }

        // 接收到hold转降落指令,将设置hold模式为false
        if(mission_cmd.pose.position.x == 88)
        {
            hold_mode = false;
        }

        switch (exec_state)
        {
            // 初始状态，等待视觉检测结果
            case WAITING_RESULT:
            {
                if(landpad_det.is_detected)
                {
                    exec_state = TRACKING;
                    message = "Get the detection result.";
                    cout << message <<endl;
                    pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, message);
                    break;
                }

                // 发送视觉节点启动指令
                vision_switch.data = true;
                vision_switch_pub.publish(vision_switch);
                
                message = "Waiting for the detection result.";
                cout << message <<endl;
                pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, message);

                // root1 adds on 270623
                if(start_move_flag==true){
                    Command_Now.header.stamp = ros::Time::now();
                    Command_Now.Mode = prometheus_msgs::ControlCommand::Move;
                    Command_Now.Command_ID = Command_Now.Command_ID + 1;
                    Command_Now.source = NODE_NAME;
                    Command_Now.Reference_State.Move_mode  = prometheus_msgs::PositionReference::XYZ_POS;
                    Command_Now.Reference_State.Move_frame = prometheus_msgs::PositionReference::ENU_FRAME;
                    Command_Now.Reference_State.position_ref[0] = start_point[0] + start_move_distance;
                    Command_Now.Reference_State.position_ref[1] = start_point[1];
                    Command_Now.Reference_State.position_ref[2] = start_point[2];
                    Command_Now.Reference_State.yaw_ref = 0.0;
                    // Command_Now.Reference_State.velocity_ref[0]  = 0.02;
                    // Command_Now.Reference_State.velocity_ref[1]  = 0.0;
                    // Command_Now.Reference_State.velocity_ref[2]  = 0.0;
                    command_pub.publish(Command_Now);
                    cout << "Moving forward ..."<<endl;
                }

                ros::Duration(1.0).sleep();
                break;
            }
            // 追踪状态
            case TRACKING:
            {
                // 丢失, 进入LOST状态
                // root1 modifies (give up)
                if(!landpad_det.is_detected && !hold_mode)
                // if(!landpad_det.is_detected && !corners_det.isdetected && !hold_mode)
                {
                    exec_state = LOST;
                    message = "Lost the Landing Pad.";
                    cout << message <<endl;
                    pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, message);
                    break;
                }   

                // 抵达上锁点, 进入LANDING
                // landpad_det.pos_body_enu_frame 为降落板在机体惯性系下的坐标
                distance_to_pad = landpad_det.pos_body_enu_frame.norm();  // ? 这个不是无人机到世界坐标系原点的距离吗
                // 达到降落距离, 上锁降落
                if(distance_to_pad < arm_distance_to_pad)
                {
                    exec_state = LANDING;
                    message = "Catched the Landing Pad.";
                    cout << message <<endl;
                    pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, message);
                    // root1 adds
                    double dt = ros::Time::now().toSec() - current_t;
                    cout << "Tracking time is " << dt << "s." << endl;
                    break;
                }
                // 达到最低高度, 上锁降落
                // else if(abs(landpad_det.pos_body_enu_frame[2]) < arm_height_to_ground)
                else if(abs(landpad_det.Detection_info.position[2]) < arm_height_to_ground)
                {
                    exec_state = LANDING;
                    message = "Reach the lowest height.";
                    cout << message <<endl;
                    pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, message);
                    // root1 adds
                    double dt = ros::Time::now().toSec() - current_t;
                    cout << "Tracking time is " << dt << "s." << endl;
                    break;
                }
                // root1 comments on 060723
                // 机体系速度控制
                /*
                Command_Now.header.stamp = ros::Time::now();
                Command_Now.Command_ID   = Command_Now.Command_ID + 1;
                Command_Now.source = NODE_NAME;
                Command_Now.Mode = prometheus_msgs::ControlCommand::Move;
                // root1 edits
                // Command_Now.Reference_State.Move_frame = prometheus_msgs::PositionReference::ENU_FRAME;
                Command_Now.Reference_State.Move_frame = prometheus_msgs::PositionReference::BODY_FRAME;
                Command_Now.Reference_State.Move_mode = prometheus_msgs::PositionReference::XYZ_VEL;
                */  
                // comments ends
                

                // root1 visp

                // store etb: drone state in the enu frame

                if(iter==0)
                {
                    cout << "Tracking starts. " << endl;
                    current_t = ros::Time::now().toSec();
                    cout << "Current time is " << current_t << endl;
                }

                iter ++;

                // velocity twist matrix cWb
                // ctb  1205 root1 change the position of tx and ty 1 - 0
                ctb[0] = camera_offset[1];  // tx  checked multiple times
                ctb[1] = camera_offset[0];  // ty
                ctb[2] = camera_offset[2];  // tz
                // cout << "ctb  = " << ctb << endl;

                // cRb
                // according to the official document
                cRb[0][0] = 0.0;
                cRb[0][1] = -1.0;
                cRb[1][0] = -1.0;
                cRb[1][1] = 0.0;
                cRb[2][2] = -1.0;
                // cout << "cRb  = " << cRb << endl;

                // velocity twist matrix bWc
                // btc
                btc[0] = camera_offset[0];  // tx
                btc[1] = camera_offset[1];  // ty
                btc[2] = camera_offset[2];  // tz
                // bRc
                bRc = cRb;

                // feed forward velocities in the ENU frame not sure about this
                // ev[0] = target_vel_xy[0];
                // ev[1] = target_vel_xy[1]；
                // this is the velocity in the diff_vehicle body frame
                // root1 test
                /* if(transformation_received)
                {          
                    int k = 0;
                    for(int i=0; i<3; i++)
                    {
                        for(int j=0; j<3; j++)
                        {
                            // cout << "i = " << i << ", j = " << j << ", k = " << k << endl;
                            cRm[i][j] = cRtm_msg.data.at(k);
                            cRm1[i][j] = landpad_det.Detection_info.rot_cRm[k];
                            k++;
                        }              
                
                    cout << "cRm from topic vs transformation is " << cRm << endl;
                    cout << "cRm from detection info is " << cRm1 << endl;
                }*/

                if(vff_received && transformation_received)
                {
                    // root1 edits
                    // get the velocity command of the ground vehicle straightly, and transform it to the camera frame
                    // then transform it to the body frame of the drone
                    // do not need to use the world frame in gazebo (in the simulation it is okay but in the real world, to get the velocity of the vehicle in the enu frame is impossible)
                    tarv[0] = target_twist.linear.x;
                    tarv[1] = target_twist.linear.y;
                    tarv[2] = target_twist.linear.z;
                    tarv[3] = target_twist.angular.x;
                    tarv[4] = target_twist.angular.y;
                    tarv[5] = target_twist.angular.z;
                    cout << "received twist in the target frame is " << tarv << endl;

                    mRtar[0][0] = 0.0;
                    mRtar[0][1] = -1.0;         
                    mRtar[1][0] = 1.0;
                    mRtar[1][1] = 0.0;
                    mRtar[2][2] = 1.0;  
                    vpVelocityTwistMatrix mWtar(mRtar); 
                    // transform the twist from the target frame to the marker frame
                    // mv = mWtar * tarv;
                    // cout << "twist in the marker frame is " << mv << endl;
                    
                    int k = 0;
                    for(int i=0; i<3; i++)
                    {
                        for(int j=0; j<3; j++)
                        {
                            // cout << "i = " << i << ", j = " << j << ", k = " << k << endl;
                            // cRm[i][j] = cRtm_msg.data.at(k);
                            cRm1[i][j] = landpad_det.Detection_info.rot_cRm[k];
                            k++;
                        }
                    }
                    // cout << "cRm from topic vs transformation is " << cRm << endl;
                    cout << "cRm from detection info is " << cRm1 << endl;
                    /*for(int i=0; i<3; i++)
                    {
                        // cout << "k2 = " << k << endl;
                        ctm[i] = cRtm_msg.data.at(k);
                        k++;
                    }*/
                    vpVelocityTwistMatrix cWm(cRm1);
                    // cout << "cRm is: " << cRm << endl;
                    // cout <<"ctm is " << ctm << endl;
                    // cout << "cWm is" << cWm << endl;
                    // transform the twist from the marker frame to the camera frame
                    // cv1 = cWm * mv;
                    // cout << "twist in the camera frame is " << cv1 << endl;

                    vpVelocityTwistMatrix bWc(bRc);
                    // transform the twist from the camera frame to the drone body frame
                    // bvt = bWc * cv1;
                    // cout << "twist in the drone frame is " << bv << endl;
                   vpVelocityTwistMatrix bWt; 
                   bWt = bWc * cWm * mWtar;
                   bvt = bWt * tarv;

                    /*ev[0] = target_twist_check.linear.x;
                    ev[1] = target_twist_check.linear.y;
                    ev[2] = target_twist_check.linear.z;
                    ev[3] = target_twist_check.angular.x;
                    ev[4] = target_twist_check.angular.y;
                    ev[5] = target_twist_check.angular.z;
                    cout << "[check] target twist in the enu frame is: " << ev << endl;
                    // must transfer the velocity from the diff_vehicle body frame to the enu frame, change it in python script?

                    // cout << "target velocities are: " << target_vel_xy[0] <<", " << target_vel_xy[1] << endl;
                    bRe = eRb.t();  // use transpose
                    bte = bRe * etb;
                    vpVelocityTwistMatrix bWe(bte, bRe);
                    // cout << "bWe is " << bWe << endl;

                    // transform the velocity of the moving target from ENU frame to body frame
                    // bvt (6, 1) velocity twist expressed in the body frame
                    // ev (6, 1) velocity twist expressed in the ENU frame
                    bv_check = bWe * ev;
                    cout << "[check] target_twist in the drone frame is: " << bv_check << endl;
                    vt[0] = bv_check[0];
                    vt[1] = bv_check[1];
                    vt[2] = bv_check[2];
                    vt[3] = bv_check[5];*/

                    // vt (4, 1) x, y, z, yaw
                    vt[0] = bvt[0];
                    vt[1] = bvt[1];
                    vt[2] = bvt[2];
                    vt[3] = bvt[5];
                    // cout << "bvt[5] =" << bvt[5] << endl;

                    cout << "vt in body frame is: " << vt << endl;
                    for(int i=0; i<4; i++)
                        vt[i] =  kvt_vs[i] * vt[i];
                }
                else  // set vt to 0, store cRm
                {
                    for(int i=0; i<4; i++)
                        vt[i] = 0.0;                  
                    // root1 adds on 190623
                    int k = 0;
                    for(int i=0; i<3; i++)
                    {
                        for(int j=0; j<3; j++)
                        {
                            // cout << "i = " << i << ", j = " << j << ", k = " << k << endl;
                            // cRm[i][j] = cRtm_msg.data.at(k);
                            cRm1[i][j] = landpad_det.Detection_info.rot_cRm[k];
                            k++;
                        }              
                    }
                }
                cout << "feed forward velocity is: " << vt << endl;

                // root1 comments on 190623
                // vpServo task;
                // vpFeaturePoint3D s, sd;
                // root1 end comments

                // root1 changes 0 - 1 (19/05/2023)
                // root1 adds vs_id (05/06/2023)
                // store coefficients in different phases
                int left_vs = 0;
                int right_vs = 0;
                // desired pose in the camera frame
                switch(vs_id){  
                        case 1:{  // leftup
                            left_vs = -1; right_vs = -1; 
                            break;
                        }                 
                        case 2: {  // leftdown
                            left_vs = -1; right_vs = 1;                    
                            break;
                        }
                        case 3: {  // rightdown
                            left_vs = 1; right_vs = 1;
                            break;
                        }
                        case 4:{  // rightup            
                            left_vs = 1; right_vs = -1;
                            break;
                        }
                        default:
                            left_vs = 0; right_vs = 0;
                            break;
                    }
                double Xd = camera_offset[1] + left_vs * len_id;
                double Yd = camera_offset[0] + right_vs * len_id;
                double Zd = camera_offset[2] + hd;
                // root1 comments on 190623
                /*
                vpPoint pointd(Xd, Yd, Zd);  // if we print this point, we have two frames: oP and cP  the desired depth need to be tuned
                vpHomogeneousMatrix cMo; // just for the form, because we already have the point in the camera frame
                cMo.buildFrom(0, 0, 0, 0, 0, 0);
                pointd.changeFrame(cMo);
                sd.buildFrom(pointd);*/
                // root1 end comments
                
                // current features from the landpad detection message
                // target position expressed in the camera frame
                double X = landpad_det.Detection_info.position[0];
                double Y = landpad_det.Detection_info.position[1];
                double Z = landpad_det.Detection_info.position[2];
                // cout<< "X" << X << "Y" << Y << "Z" << Z << endl;
                //  root1 comments on 190623
                /*
                vpPoint point(X, Y, Z);
                point.changeFrame(cMo);

                s.buildFrom(point);
                cout << "s is" << endl;
                s.print();
                cout << "-------------------------------------------" << endl;

                task.addFeature(s, sd);*/
                // root1 end comments
                // root1 adds on 190623
                // double ThetaUx = landpad_det.Detection_info.tu[0];
                // double ThetaUy = landpad_det.Detection_info.tu[1];
                // double ThetaUz = landpad_det.Detection_info.tu[2];
                vpRotationMatrix mRcd{1.0, 0, 0, 0, -1.0, 0, 0, 0, -1.0};
               // cRm1 is already computed before. type - vpRotationMatrix
               vpRotationMatrix cRcd1 = cRm1 * mRcd;  // current cRcd
               cRcd1 = cRcd1.inverse();
               // cout << "cRcd1 = " << cRcd1 << endl;

                vpFeatureTranslation trans_vs(vpFeatureTranslation::cMo);  // current t
                // vpFeatureThetaU thetau_vs(vpFeatureThetaU::cRcd);  // current tu
                vpFeatureThetaU thetau_vs(vpFeatureThetaU::cdRc);
                trans_vs.set_Tx(X);
                trans_vs.set_Ty(Y);
                trans_vs.set_Tz(Z);
                // thetau_vs.set_TUx(ThetaUx);
                // thetau_vs.set_TUy(ThetaUy);
                // thetau_vs.set_TUz(ThetaUz);
                thetau_vs.buildFrom(cRcd1);

                vpFeatureTranslation transd_vs(vpFeatureTranslation::cMo);  // desired t
                // vpFeatureThetaU thetaud_vs(vpFeatureThetaU::cRcd);  // desired tu
                vpFeatureThetaU thetaud_vs(vpFeatureThetaU::cdRc);
                transd_vs.set_Tx(Xd);
                transd_vs.set_Ty(Yd);
                transd_vs.set_Tz(Zd);
                vpRotationMatrix cRcd_desired{1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0};
                // thetaud_vs.set_TUx(0.0);
                // thetaud_vs.set_TUy(0.0);
                // thetaud_vs.set_TUz(0.0);
                thetaud_vs.buildFrom(cRcd_desired);

                vpServo task;
                task.addFeature(trans_vs, transd_vs);
                task.addFeature(thetau_vs, thetaud_vs);

                task.setServo(vpServo::EYEINHAND_L_cVe_eJe);
                task.setInteractionMatrixType(vpServo::CURRENT);
                // task.setLambda(lambda); 
                vpAdaptiveGain lambda_a(lambda_0, lambda_inf, 30);
                // use adaptive gain
                task.setLambda(lambda_a); 
                cout << "current gain is: " << lambda_a << endl;
                // task.setLambda(0.6);

                // Jacobian matrix J (6, 4)
                J1[0][0] = 1; J1[1][1] = 1; J1[2][2] = 1; J1[5][3] = 1;
                // J (6, 3)
                //J[0][0] = 1; J[1][1] = 1; J[2][2] = 1;
                //task.set_eJe(J);
                task.set_eJe(J1);  // Set the robot jacobian expressed in the end-effector frame.

                vpVelocityTwistMatrix cWb(ctb, cRb);
                // cout << "velocity twist matrix is " << cWb << endl;
                task.set_cVe(cWb);

                u = task.computeControlLaw() +  vt;
                // cout << "u without vt = " << task.computeControlLaw() << endl;
                // cout << "vt = " << vt << endl;
                e = task.getError();  // e = s - s*
                cout << "error is " << e << endl;

               // vs_error_x.data = e[0];
               // vs_error_y.data = e[1];
               // vs_error_z.data = e[2];
               // cout << typeid(e[0]).name() << endl;  // e[0] is double, checked

                // vsError_x_pub.publish(vs_error_x);
                // vsError_y_pub.publish(vs_error_y);
                // vsError_z_pub.publish(vs_error_z);

                vs_error.data.push_back(e[0]);
                vs_error.data.push_back(e[1]);
                vs_error.data.push_back(e[2]);
                // root1 adds on 190623
                vs_error.data.push_back(e[3]);
                vs_error.data.push_back(e[4]);
                vs_error.data.push_back(e[5]);

               vsError_pub.publish(vs_error);
                cout << "u = " << u << endl;
                // root1 test tracking
                // u[0] = bvt[0];
                // u[1] = bvt[1];
                // u[2] = bvt[2];
                // u[3] = bvt[5];
                // root1 test feedforward velocity
                // u = 0;
                // u[0] = 1.0;
                // u[1] = 0.0;
                // u[2] = 0.0;
                // u[3] = 0.0;
                cout << endl;
                cout << "===================================" << endl;

                // root1 adds on 060723
                // 机体系速度控制
                Command_Now.header.stamp = ros::Time::now();
                Command_Now.Command_ID   = Command_Now.Command_ID + 1;
                Command_Now.source = NODE_NAME;
                Command_Now.Mode = prometheus_msgs::ControlCommand::Move;
                // root1 edits
                // Command_Now.Reference_State.Move_frame = prometheus_msgs::PositionReference::ENU_FRAME;
                Command_Now.Reference_State.Move_frame = prometheus_msgs::PositionReference::BODY_FRAME;
                Command_Now.Reference_State.Move_mode = prometheus_msgs::PositionReference::XYZ_VEL;  
                Command_Now.Reference_State.Yaw_Rate_Mode = true;  

                for(int i=0; i<3; i++){
                    Command_Now.Reference_State.velocity_ref[i] = u[i] * k_vs[i];
                    if(force_landing_flag==true && Z<=0.35)
                    {
                        cout << "force speed up" << endl;
                        Command_Now.Reference_State.velocity_ref[2] = -0.15;
                    }
                }  
                // root1 adds on060723               
                Command_Now.Reference_State.yaw_rate_ref  = u[3] * k_vs[3];
                // test 
                // Command_Now.Reference_State.velocity_ref[2] = 0.0;
                // cout << "yaw_rate_ref = " << u[3] * k_vs[3] << endl;
                // Command_Now.Reference_State.yaw_rate_ref  = vt[3];
                // 100124 check the value of the reference yaw rate
                cout << "yaw_rate_ref = " << Command_Now.Reference_State.yaw_rate_ref << endl;
                 
                /*vc[0] = 1;
                vc[1] = 1;
                vc[2] = 1;
                vc[3] = 0;
                vc[4] = 0;
                vc[5] = 1;
                // update pose
                M = vpExponentialMap::direct(vc) * M;
                // update pose vector
                pose.buildFrom(M);
                // log
                logger.update();*/

                // root1 tests
                // Command_Now.Reference_State.velocity_ref[2] = 0.0;
                // for(int i=0; i<3; i++)
                    // Command_Now.Reference_State.velocity_ref[i] = vt[i];
                
                // Command_Now.Reference_State.yaw_rate_ref = vt[3];

                // root1 comments
                // Command_Now.Reference_State.yaw_ref  = 0.0;
                
                //Publish

                if (!hold_mode)
                {
                    command_pub.publish(Command_Now);
                }

                break;
            }
            case LOST:
            {
                cout << "Lost. " << endl;
                static int lost_time = 0;
                lost_time ++ ;
                
                // 重新获得信息,进入TRACKING
                if(landpad_det.is_detected)
                {
                    exec_state = TRACKING;
                    lost_time = 0;
                    message = "Regain the Landing Pad.";
                    cout << message <<endl;
                    pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, message);
                    break;
                }   
                
                // 首先是悬停等待 尝试得到图像, 如果仍然获得不到图像 则原地上升
                if(lost_time < 10.0)
                {
                    Command_Now.header.stamp = ros::Time::now();
                    Command_Now.Command_ID   = Command_Now.Command_ID + 1;
                    Command_Now.source = NODE_NAME;
                    Command_Now.Mode = prometheus_msgs::ControlCommand::Hold;

                    ros::Duration(0.4).sleep();
                }else
                {
                    Command_Now.header.stamp  = ros::Time::now();
                    Command_Now.Mode = prometheus_msgs::ControlCommand::Move;
                    Command_Now.Command_ID  = Command_Now.Command_ID + 1;
                    Command_Now.source  = NODE_NAME;
                    Command_Now.Reference_State.Move_mode  = prometheus_msgs::PositionReference::XYZ_VEL;
                    Command_Now.Reference_State.Move_frame  = prometheus_msgs::PositionReference::BODY_FRAME;
                    Command_Now.Reference_State.velocity_ref[0]  = 0.0;
                    Command_Now.Reference_State.velocity_ref[1]  = 0.0;
                    Command_Now.Reference_State.velocity_ref[2]  = 0.1;
                    Command_Now.Reference_State.yaw_ref  = 0;

                    // 如果上升超过原始高度，则认为任务失败，则直接降落
                    if(_DroneState.position[2] >= start_point[2])
                    {
                        exec_state = LANDING;
                        lost_time = 0;
                        message = "Mission failed, landing... ";
                        cout << message <<endl;
                        pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, message);
                        break;
                    }
                }
                command_pub.publish(Command_Now);
                break;
            }
            case LANDING:
            {
                if(sim_mode)    // simulation
                {
                    cout << "Sim mode" << endl;
                    Command_Now.header.stamp = ros::Time::now();
                    Command_Now.Command_ID   = Command_Now.Command_ID + 1;
                    Command_Now.source = NODE_NAME;
                    // Command_Now.Mode = prometheus_msgs::ControlCommand::Disarm; // 新飞控不支持直接上锁,会变成返航模式
                    Command_Now.Mode = prometheus_msgs::ControlCommand::Land;
                    command_pub.publish(Command_Now);
                }else  // real experiment
                {
                    cout << "Real world mode" << endl;
                    Command_Now.header.stamp = ros::Time::now();
                    Command_Now.Command_ID   = Command_Now.Command_ID + 1;
                    Command_Now.source = NODE_NAME;
                    Command_Now.Mode = prometheus_msgs::ControlCommand::Land;
                    command_pub.publish(Command_Now);
                }

                ros::Duration(1.0).sleep();

                break;
            }
        }
        
        // Your node's code goes here.
        //cout << "hello, vs." << endl;

        rate.sleep();
    }
}

void printf_result()
{

    cout << ">>>>>>>>>>>>>>>>>>>>>>Autonomous Landing Mission<<<<<<<<<<<<<<<<<<<"<< endl;

    switch (exec_state)
    {
        case WAITING_RESULT:
            cout << "exec_state: WAITING_RESULT" <<endl;
            break;
        case TRACKING:
            cout << "exec_state: TRACKING" <<endl;
            break;
        case LOST:
            cout << "exec_state: LOST" <<endl;
            break;
        case LANDING:
            cout << "exec_state: LANDING" <<endl;
            break;
    } 

    cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>Vision State<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    if(landpad_det.is_detected)
    {
        cout << "is_detected: true" <<endl;
    }else
    {
        cout << "is_detected: false" <<endl;
    }
    
    cout << "Target_pos (body): " << landpad_det.pos_body_frame[0] << " [m] "<< landpad_det.pos_body_frame[1] << " [m] "<< landpad_det.pos_body_frame[2] << " [m] "<<endl;

    cout << "Target_pos (body_enu): " << landpad_det.pos_body_enu_frame[0] << " [m] "<< landpad_det.pos_body_enu_frame[1] << " [m] "<< landpad_det.pos_body_enu_frame[2] << " [m] "<<endl;

    cout << "Ground_truth(pos):  " << GroundTruth.pose.pose.position.x << " [m] "<< GroundTruth.pose.pose.position.y << " [m] "<< GroundTruth.pose.pose.position.z << " [m] "<<endl;
    cout << "Detection_ENU(pos): " << landpad_det.pos_enu_frame[0] << " [m] "<< landpad_det.pos_enu_frame[1] << " [m] "<< landpad_det.pos_enu_frame[2] << " [m] "<<endl;
    cout << "Detection_ENU(yaw): " << landpad_det.att_enu_frame[2]/3.1415926 *180 << " [deg] "<<endl;

    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>Land Control State<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    cout << "vel_cmd: " << Command_Now.Reference_State.velocity_ref[0] << " [m/s] "<< Command_Now.Reference_State.velocity_ref[1] << " [m/s] "<< Command_Now.Reference_State.velocity_ref[2] << " [m/s] "<<endl;
}

void printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "hold_mode : "<< hold_mode << endl;
    cout << "sim_mode : "<< sim_mode << endl;
    cout << "use_pad_height : "<< use_pad_height << endl;
    
    cout << "arm_distance_to_pad : "<< arm_distance_to_pad << endl;
    cout << "arm_height_to_ground : "<< arm_height_to_ground << endl;

    cout << "hd: " << hd << endl;

    cout << "lambda_0: " << lambda_0 << endl;
    cout << "lambda_inf: " << lambda_inf << endl;

    cout << "kx_vs : "<< k_vs[0] << endl;
    cout << "ky_vs : "<< k_vs[1] << endl;
    cout << "kz_vs : "<< k_vs[2] << endl;
    cout << "kw_vs : "<< k_vs[3] << endl;

    cout << "kvtx_vs: " << kvt_vs[0] << endl;
    cout << "kvty_vs: " << kvt_vs[1] << endl;
    cout << "kvtz_vs: " << kvt_vs[2] << endl;
    cout << "kvtw_vs: " << kvt_vs[3] << endl;

    // cout << "lambda: " << lambda << endl;

    cout << "start_point_x : "<< start_point[0] << endl;
    cout << "start_point_y : "<< start_point[1] << endl;
    cout << "start_point_z : "<< start_point[2] << endl;
    
    cout << "camera_offset_x : "<< camera_offset[0] << endl;
    cout << "camera_offset_y : "<< camera_offset[1] << endl;
    cout << "camera_offset_z : "<< camera_offset[2] << endl;
    
    // cout << "target_vel_x : "<< target_vel_xy[0] << endl;
    // cout << "target_vel_y : "<< target_vel_xy[1] << endl;
}
