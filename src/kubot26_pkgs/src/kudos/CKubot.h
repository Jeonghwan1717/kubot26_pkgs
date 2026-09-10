#ifndef CKUBOT_H_
#define CKUBOT_H_


//* Basic header, Library
#include <stdio.h>
#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <algorithm>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <kubot26_pkgs/msg/kubot_control_msgs.hpp>
#include <sensor_msgs/msg/joint_state.hpp>


#include <Eigen/Dense>
#include <Eigen/Core>


using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::MatrixXf;
using Eigen::VectorXf;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Vector4d;


using Eigen::Vector2f;
using Eigen::Vector3f;
using Eigen::Vector4f;


#define Qp_N 30


#define PI      3.141592
#define D2R     PI/180.
#define R2D     180./PI 

#define onesecSize  1000
#define tasktime    0.001

#define PatternPlan 4
#define PatternElement 6

#define MPC_GRF_N 5 // Control Horizon N for Model Predictive Control based Ground Reaction Force Control 


//* Command flag mode

typedef enum {
    NONE_ACT,
    RETURN_HOMEPOSE,
    GOTO_WALK_READY_POS,

    UPDOWN,
    SWAYMOTION,
    SWAYMOTION_PATTERN,
    KICK_PATTERN,
    KICKTEST,

    // Walking
    ACT_DSP,
    ACT_SSP,
    ACT_SSP_Mode,
    ACT_TEST_WALK,
    ACT_TESTING_WALK,
    ACT_STOPPING_WALK,
    ACT_STOP_WALK,
    ACT_START_WALK,
    ACT_STOPPING_PATTERN_WALK,
    ACT_CHANGE_WALK,
    ACT_INF_WALK,
    ACT_PATTERN_TEST,
    ACT_PATTERN_TESTING,
    ACT_SIDE_PATTERN_TEST,
    ACT_TURN_PATTERN_TEST,

    //uppper body test
    ACT_UPPER_TEST,
    ACT_SWING_ARMS,
    ACT_STANDUP_FRONT,
    ACT_STANDUP_BACK,

    //motion
    ACT_HURDEL,
    ACT_BALLCATCH,
    ACT_BALLSWING,
    
    Raise_Left_Arm,
    WALKTHREETIMES,
    kickreadypose,
    KICKTHEBALL,
    GOALKEEPER,
    walkingphysics,
    
    ACT_KICK_L,
    ACT_KICK_R,
    ACT_TURNING_L,
    ACT_TURNING_R,
   

} _COMMAND_FLAG;

//* Control Mode

typedef enum {
    CTRLMODE_NONE,
    CTRLMODE_HOMEPOSE,
    CTRLMODE_WALKREADY,
    CTRLMODE_WALKING,
    CTRLMODE_WALKING_TEST,
    

    CTRLMODE_UPDOWN,
    CTRLMODE_SWAY,
    CTRLMODE_KICKTEST,

    CTRLMODE_UPPER_TEST,
    CTRLMODE_STANDUP_TEST,

    CTRLMODE_SRCMOTION,
    CTRLMODE_RAISELEFTARM,
    CTRLMODE_WALKWALKWALK,
    CTRLMODE_KICKREADYPOSE,
    CTRLMODE_MOTION,
    CTRLMODE_GOALKEEPERMODE,
    CTRLMODE_PHYSICSMODE,
    CTRLMODE_TURNING_L,
    CTRLMODE_TURNING_R,
    


} _CONTROL_MODE;

// bool initposition; //* InitPosition


//* struct for Joints and End Points variables

typedef struct Joint {
    //* Current information

    float currentAngle; // [rad]
    float currentVel; // [rad/s]
    float currentAcc; // [rad/s^2]
    float currentTorque; // [N·m]

    //* Reference information

    // float refAngle; //[rad]
    float refVel; // [rad/s]
    float refAcc; // [rad/s^2]
    float refTorque; // [N·m]
    float ctcTorque; // [N·m]

    float compensate_refAngle; // [rad]

    float walkReadyAngle; // [rad]

    float gain_scheme; // changed by pattern scheme
} JOINT;

typedef struct Base //coordinate of Base
{
    //* Current information
    VectorXd current; //* EndPoint's Current Position
    VectorXd vel;     //* EndPoint's Current Velocity
    VectorXd acc;     //* EndPoint's Current Acceleration
    VectorXd pre;     //* EndPoint's Pre Position
    VectorXd prevel;  //* EndPoint's Pre Position
    VectorXd preacc;  //* EndPoint's Pre Position

//* Reference information
    VectorXd refpos;     //* EndPoint's ref Position
    VectorXd refvel;     //* EndPoint's ref Velocity
    VectorXd refacc;     //* EndPoint's ref acceleration
    VectorXd refquat;    //* EndPoint's ref quat 
    VectorXd prerefpos;  //* EndPoint's ref Position
    VectorXd prerefvel;  //* EndPoint's ref Velocity
    VectorXd prerefacc;  //* EndPoint's ref Acceleration

    int ID;

} BASE;

typedef struct Leg_Kinematics {
    //*Humanoid Lower Body Kinematics
    Vector3d Global_to_Base;
    Vector3d Base_to_Hip_y;
    Vector3d Hip_y_to_Hip_p;
    Vector3d Hip_y_to_Hip_p_dot;
    Vector3d Hip_to_Knee;
    Vector3d Hip_p_to_Knee;
    Vector3d Knee_to_Ankle;
    Vector3d Ankle_p_to_Ankle_r; // Ankle pitch To Ankle roll
    Vector3d Ankle_p_to_Ankle_rd; // Ankle pitch To Ankle roll Dot
    Vector3d Ankle_to_Foot; // Ankle roll To foot
    Vector3d Ankle_rd_to_Foot; // Ankle roll Dot To Foot
    Vector3d Ankle_r_to_Ankle_rd; // Ankle roll To Ankle roll dot
    Vector3d Base_to_Foot;

    float m_Hip_yaw_link;
    float m_Hip_roll_pitch_link;
    float m_Thigh_link;
    float m_Calf_link;
    float m_Ankle_pitch_link;
    float m_Ankle_roll_and_Foot_link;
    float m_Foot;

    Vector3d c_Hip_yaw_link;
    Vector3d c_Hip_roll_pitch_link;
    Vector3d c_Thigh_link;
    Vector3d c_Calf_link;
    Vector3d c_Ankle_pitch_link;
    Vector3d c_Ankle_roll_and_Foot_link;
} LEG;

typedef struct LowerbodyKine {
    //*B_kubot Body Kinematics 단위는 mm
    double BASE_TO_LEG_Y;// = 50; 
    double BASE_TO_LEG_Z;// = 104.64; 
    double THIGH_LENGTH;// = 138.0; 
    double CALF_LENGTH;// = 143.0; 
    double ANKLE_LENGTH;// = 40; 
    double LEG_LENGTH;// = THIGH_LENGTH + CALF_LENGTH + ANKLE_LENGTH; //mm (THIGH_LENGTH + CALF_LENGTH + ANKLE_LENGTH)
    double LEG_SIDE_OFFSET;// = BASE_TO_LEG_Y; // BASE_TO_LEG_Y
    // End point ~ base : 425.64mm

    //*B_kubot Body Kinematics

    double foot_x_length ;//= 150.0; //*Foot x length
    double foot_y_length ;//= 100.0; //*Foot y length
    //*Humanoid Lower Body Kinematics
    LEG L, R;
} LBK;

typedef struct UpperbodyKine {
    //*Humanoid Upper Body Kinematics
    Vector3d Base_to_Torso;

    float m_Upper_body_link;
    Vector3d c_Upper_body_link;
} UBK;

typedef struct bodyKine {
    //*Humanoid Upper Body Kinematics
    LBK lower; // Humanoid Robots Lower Body Kinematics
    UBK upper;

    float m_base_link;
    float m_total;
    // float** G_I_tensor = matrix(1, 3, 1, 3); // G_I_tensor is the inertia tensor in Global coordinates.
    // float** inv_G_I_tensor = matrix(1, 3, 1, 3); // inv_G_I_tensor is the inverse matrix to G_I_tensor.    
    // float** B_I_tensor = matrix(1, 3, 1, 3); // B_I_tensor is the inertia tensor in Body coordinates.
    Vector3d c_base_link;

} BodyKine;

typedef struct EndPoint {
    //* Current information
    VectorXd cur_G_pos; // EndPoint's Current Position & Orientation in Global frame coordinate system
    VectorXd cur_G_vel; // EndPoint's Current Velocity & Angular Velocity in Global frame coordinate system
    VectorXd cur_G_acc; // EndPoint's Current Acceleration & dot Angular Velocity in Global frame coordinate system

    VectorXd cur_B_pos; // EndPoint's Current Position & Orientation in Base frame coordinate system
    VectorXd cur_B_vel; // EndPoint's Current Velocity & Angular Velocity in Base frame coordinate system
    VectorXd cur_B_acc; // EndPoint's Current Acceleration & dot Angular Velocity in Base frame coordinate system    

    //* Reference information
    VectorXd ref_G_pos; // EndPoint's ref Position & Orientation in Global frame coordinate system
    VectorXd ref_G_pattern_pos; // EndPoint's ref Position & Orientation pattern pos in Global frame coordinate system
    VectorXd ref_G_vel; // EndPoint's ref Velocity & Angular Velocity in Global frame coordinate system
    VectorXd ref_G_acc; // EndPoint's ref acceleration & dot Angular Velocity in Global frame coordinate system
    Vector4d ref_G_quat; // EndPoint's ref quat in Global frame coordinate system

    VectorXd ref_B_pos; // EndPoint's ref Position & Orientation in Base frame coordinate system
    VectorXd ref_B_vel; // EndPoint's ref Velocity & Angular Velocity in Base frame coordinate system
    VectorXd ref_B_acc; // EndPoint's ref acceleration & dot Angular Velocity in Base frame coordinate system
    Vector4d ref_B_quat; // EndPoint's ref quat in Base frame coordinate system

    // float* ref_G_CoM_to_Foot = fvector(1, 3); // reference vector from the CoM to the endpoints (each foot) in Global frame coordinate system, [m]
    // float* cur_G_CoM_to_Foot = fvector(1, 3); // current vector from the CoM to the endpoints (each foot) in Global frame coordinate system, [m]
    // float* ref_G_CoM_to_vel = fvector(1, 3); // differential reference vector from the CoM to the endpoints (each foot) in Global frame coordinate system, [m/s]
    // float* cur_G_CoM_to_vel = fvector(1, 3); // differential current vector from the CoM to the endpoints (each foot) in Global frame coordinate system, [m/s]

    //* Current information
    VectorXd current; //Vector6f current; // EndPoint's Current Position
    VectorXd vel; //Vector6f vel; // EndPoint's Current Velocity
    VectorXd acc; //Vector6f acc; // EndPoint's Current Acceleration
    VectorXd pre; //Vector6f pre; // EndPoint's Pre Position
    VectorXd prevel; ///Vector6f prevel; // EndPoint's Pre Velocity
    VectorXd preacc; //Vector6f preacc; // EndPoint's Pre Accelerationn

    //* Reference information
    VectorXd refpos; // EndPoint's ref Position
    VectorXd refvel; // EndPoint's ref Velocity
    VectorXd refacc; // EndPoint's ref acceleration
    Vector4d refquat; // EndPoint's ref quat
    VectorXd prerefpos; // EndPoint's ref Position
    VectorXd prerefvel; // EndPoint's ref Velocity
    VectorXd prerefacc; // EndPoint's ref Acceleration

    VectorXd FTS; // Fx, Fy, Fz, Mx, My, Mz of FT sensor

    int ID; // ID of each link at URDF file

} ENDPOINT;

typedef struct XYSystemID // systemID for x,y-axis
{
    MatrixXd T1;     //SystemID for ZMPobserver
    MatrixXd T2;     //SystemID for ZMPobserver
    MatrixXd T1_DSP; //SystemID for ZMPobserver
    MatrixXd T2_DSP; //SystemID for ZMPobserver
    MatrixXd T1_SSP; //SystemID for ZMPobserver
    MatrixXd T2_SSP; //SystemID for ZMPobserver
    double T;        //SystemID for ZMPobserver

    double Spring_K;  //SystemID K for ZMPobserver
    double Damping_C; //SystemID C for ZMPobserver

    double Spring_K_DSP;  //SystemID K for ZMPobserver
    double Damping_C_DSP; //SystemID C for ZMPobserver

    double Spring_K_SSP;  //SystemID K for ZMPobserver
    double Damping_C_SSP; //SystemID C for ZMPobserver

    MatrixXd A; //SystemID for ZMPobserver
    MatrixXd B; //SystemID for ZMPobserver
    MatrixXd C; //SystemID for ZMPobserver
    double   D; //SystemID for ZMPobserver

    MatrixXd A_DSP; //SystemID for ZMPobserver
    MatrixXd B_DSP; //SystemID for ZMPobserver
    MatrixXd C_DSP; //SystemID for ZMPobserver
    double   D_DSP; //SystemID for ZMPobserver

    MatrixXd A_SSP; //SystemID for ZMPobserver
    MatrixXd B_SSP; //SystemID for ZMPobserver
    MatrixXd C_SSP; //SystemID for ZMPobserver
    double   D_SSP; //SystemID for ZMPobserver


} XYID;

typedef struct SystemID //SystemID
{
    double ComHeight; //mm /
    double robotM;    //kg //
    double robotI;    //Moment of Inertia
    double robotH;    //now Robot CoM height
    double robotw;    //now Robot natural frequency

    XYID X, Y;

} ID;

typedef struct XYPreviewControl // PreviewControl for x-axis
{
    MatrixXd ref ;
    double m_ref ;

    double old_zmp;
    MatrixXd state;     //3X1 matrix
    MatrixXd new_state; //3X1 matrix
    double E;
    double sum_E;
    double sum_P;
    double U;
    double CoM, dCoM;

} XYPRE;

typedef struct PreviewControl //
{
    double RefTotalTrajSize;
    double RefTotalTime;
    double PreviewTimeSize;
    double PeriodTimeSize;
    double PreviewTime;
    double PreviewReadyTime;

    MatrixXd A;

    MatrixXd B;

    MatrixXd C;

    MatrixXd B_Tilde;

    MatrixXd I_Tilde;

    MatrixXd F_Tilde;

    MatrixXd Q_Tilde;

    MatrixXd R;

    MatrixXd A_Tilde;

    MatrixXd K_Tilde;

    MatrixXd G_I;

    MatrixXd G_X;

    MatrixXd G_P;

    MatrixXd A_Tilde_c;

    MatrixXd X_Tilde;

    XYPRE X, Y;

    int count;
    int setcount;

} PREVIEWCONTROL;


typedef struct XYObserver // Observer for y-axis
{
    double m_hat_state;         //* ZMPobserver
    MatrixXd hat_state;         //* ZMPobserver hat state
    MatrixXd hat_new_state;     //* ZMPobserver new hat state
    MatrixXd hat_d_state;       //* ZMPobserver hat d state

    MatrixXd dsp_hat_state;     //* ZMPobserver dsp hat state
    MatrixXd dsp_hat_new_state; //* ZMPobserver dsp new hat state
    MatrixXd dsp_hat_d_state;   //* ZMPobserver dsp hat d state

    MatrixXd ssp_hat_state;     //* ZMPobserver ssp hat state
    MatrixXd ssp_hat_new_state; //* ZMPobserver ssp new hat state
    MatrixXd ssp_hat_d_state;   //* ZMPobserver ssp hat d state

    double hat_ZMP;             //* ZMPobserver hay zmp
    double m_hat_ZMP;           //* ZMPobserver
    double dsp_hat_ZMP;         //* ZMPobserver dsp hat zmp
    double ssp_hat_ZMP;         //* ZMPobserver ssp hat zmp

    double m_disturbance;       //* ZMPobserver

    MatrixXd L;   //ZMPobserver gain L

    MatrixXd L_DSP; //ZMPobserver gain L
    MatrixXd L_SSP; //ZMPobserver gain L

} XYOBSERVER;


typedef struct Observer //
{
    XYOBSERVER X, Y;

    enum {
        NOdisturbance = 0,
        Disturbance = 1,
        Slope = 2 //start foot L
    };

    int disturbanceExist;

    Eigen::Vector3d groundAngle;
    Eigen::Vector3d groundVel;
    Eigen::Vector3d groundpreAngle;

} OBSERVER;

typedef struct XControl // Observer for x-axis
{
    double U; //ZMPControl Input U
    double dsp_U; //ZMPControl Input U
    double ssp_U; //ZMPControl Input U
    double m_U; //ZMPControl Input U
    double disturbanceRejection_U; //ZMPControl Input disturbance Rejection
    double dsp_disturbanceRejection_U; //ZMPControl Input disturbance Rejection
    double ssp_disturbanceRejection_U; //ZMPControl Input disturbance Rejection
    double slope_compensate; //Input Compensate slope
    double U_Limit; //ZMPControl Input U

    MatrixXd K; //ZMPControl gain K
    MatrixXd K_DSP; //ZMPControl gain K
    MatrixXd K_SSP; //ZMPControl gain K

    //JUMPREADY by YSJ
    double jump_U;
    double jump_Error;

    double jump_U_dot;
    double jump_U_old;

} XCONTROL;

typedef struct YControl // Observer for y-axis
{
    double U; //ZMPControl Input U
    double dsp_U; //ZMPControl Input U
    double ssp_U; //ZMPControl Input U
    double m_U; //ZMPControl Input U
    double disturbanceRejection_U; //ZMPControl Input disturbance Rejection
    double dsp_disturbanceRejection_U; //ZMPControl Input disturbance Rejection
    double ssp_disturbanceRejection_U; //ZMPControl Input disturbance Rejection
    double slope_compensate; //Input Compensate slope
    double U_Limit; //ZMPControl Input U

    MatrixXd K; //ZMPControl gain K
    MatrixXd K_DSP; //ZMPControl gain K
    MatrixXd K_SSP; //ZMPControl gain K
} YCONTROL;

typedef struct Controller //
{
    XCONTROL X;
    YCONTROL Y;
    bool dsp2ssp;
    bool ssp2dsp;
    bool gainchangeflag;
    double gainChangeTime;
    double gainChange_t;

    double dsp_gain;
    double ssp_gain;

    Eigen::Vector3d U;

    bool ON;
    bool OFF;

} CONTROL;

typedef struct zmpx //
{
    double sensor, old, init;
    double Fmargin;
    double Bmargin;

} XZMP;

typedef struct zmpy //
{
    double sensor, old, init;
    double Lmargin;
    double Rmargin;

} YZMP;

 typedef struct ZeroMomentPoint //ZMP
{
    PREVIEWCONTROL previewControl;
    OBSERVER observer;
    CONTROL controller;

    Eigen::Vector3d global_ref;
    Eigen::Vector3d global;
    Eigen::Vector3d local;

    XZMP X;
    YZMP Y;

} ZMP;

typedef struct Step //Step
{
    int cur_num, cur_num_footprints;
    int total_num, total_footprints;

    double FBstepSize,   oldFBstepSize;
    double LRstepSize,   oldLRstepSize;
    double TurnSize,     oldTurnSize;
    double footHeight;
    double turningPoint;

    enum {
        S_L_F = 1,  //start foot L
        S_R_F = -1, //start foot R
        L_L_F = 1,  //last foot L
        L_R_F = -1  //last foot R
    };

    int start_swing_foot; //start foot
    int last_swing_foot;

    enum {
        forwardWalking = 1, //walk forward
        backwardWalking = -1, // back forward
        leftWalking = 1, //left
        rightWalking = -1, // right
        leftTurning = 1,
        rightTurning = -1,
        TurnModeOn =1,
        TurnModeOff = 0,
        WalkinginPlace = 0 //
    };
    //    static int None = 0;
    int walking_f_or_b; //walkig f or b?
    int walking_l_or_r; //walkig l or r?
    int walking_lT_or_rT; //walkig l or r?
    int walking_Turn;

    MatrixXd LFoot, RFoot;
    VectorXd preLfoot, preRfoot, prefootCenter;
    VectorXd LastLfoot, LastRfoot, LastfootCenter;

} STEP;

typedef struct WalkingTime //Step
{
    double sec; //*time while robot is walking
    double wsec;
    double old_sec;
    double init_sec;

    double periodTime; //*walking periodTime
    double DSP_ratio; //*DSP ratio of walking periodTime
    double SSP_ratio; //*SSP ratio of walking periodTime
    double SSP_start_time;
    double SSP_end_time;
    double SSP_time;


    double XYDSP_ratio; //*DSP ratio of walking XY pattern periodTime
    double XYSSP_ratio; //*SSP ratio of walking XY pattern periodTime
    double XYSSP_start_time;
    double XYSSP_end_time;
    double XYSSP_time;
    double readytime;

} WALKINGTIME;

typedef struct Walking //
{

    STEP step;
    WALKINGTIME time;
    // SWINGLEGVIBRATION SwingLegVibrationCon;
    // LANDINGCON Landing;

    enum \
     {
        LSSP = 1,
        RSSP = -1,
        DSP = 0,
        SSP = 2,
        UP = 3
    };
//////////////0521//////////////
    int Pattern_SP; //Support Phase determined by Walking Pattern Scheme 
    int Pattern_SSP; //Single Support Phase determined by Walking Pattern Scheme
////////////////////////////////

    int ft_SP; //* SP by FTsensor
    int old_ft_SP; //* SP by FTsensor

    int walk_SP; //* SP by WalkingPattern
    int walk_SSP;
    int old_walk_SP; //* SP by WalkingPattern
    int old_walk_SSP; //* SSP by WalkingPattern

    int SP; //* Checking SP by FTsensor, WalkingPattern
    int old_SP; //* Checking SP by FTsensor, WalkingPattern

    int SP_check; //* SP for Checking
    int SP_oldcheck; //* SP for Checking
    int SP_checking; //* check number for Checking
    int Mode;

    bool change_FB_walking_flag;
    bool change_LR_walking_flag;
    bool change_Turn_flag;
    bool change_LTurn_flag;
    bool change_RTurn_flag;


    bool Ready;
    bool walk;
    bool ssp_mode;

    bool NewfootPlan;

    bool initialize;
} WALK;

typedef struct Quadratic_programming//
{
//    Quadratic Programming
//    Based on 'Trajectory Free Linear Model Predictive Control for Stable Walking in the Presence of Strong Perturbations
//    - Pierre-Brice Wieber -
//    1/2 xHx+xf

    MatrixXd A;
    MatrixXd B;
    MatrixXd C;
    
    MatrixXd Px;
    MatrixXd Pu;
    
    MatrixXd H; //H=Q*Pu'*Pu+R*I
    MatrixXd G; //[Pu];
    
    MatrixXd f;
    MatrixXd ref_ZMP_traj;
    MatrixXd zmp_by_com;
 
    VectorXd max_zmpx;
    VectorXd min_zmpx;
    VectorXd max_zmpy;
    VectorXd min_zmpy;

    
    Eigen::Vector3d m_max_zmp_margin;
    Eigen::Vector3d m_min_zmp_margin;
    
    Eigen::Vector3d m_ref_zmp;
    Eigen::Vector3d m_ref_com;
    Eigen::Vector3d m_zmp_by_com;

    
    double Q;
    double R;
    
    int N; //the number N of pieces of the trajectory
    int total_N; //the total N of pieces of the every trajectory
    double T_ref_traj; //sec, time of reference trajectory
    double Test_time_traj; //sec, time of All reference trajectory
    double T; //sampling time
    int count;

    float H_x[(Qp_N+1)*Qp_N/2];
    int   H_nnz = (Qp_N+1)*Qp_N/2;
    int   H_i[(Qp_N+1)*Qp_N/2];
    int   H_p[(Qp_N+1)];
    
    float q_x[Qp_N];
    float q_y[Qp_N];


    float A_x[(Qp_N+1)*Qp_N/2];
    int   A_nnz = (Qp_N+1)*Qp_N/2;
    int   A_i[(Qp_N+1)*Qp_N/2];
    int   A_p[(Qp_N+1)];
    
    float zmpxl[Qp_N];
    float zmpxu[Qp_N];
    float zmpyl[Qp_N];
    float zmpyu[Qp_N];
    
    int n = Qp_N;
    int m = Qp_N;
    
    int exitflag_for_x;
    int exitflag_for_y;
    
    MatrixXd state; //3X3 Matrix, col(1) = x, dx, ddx, col(2) = y, dy, ddy, col(3) = z, dz, ddz
    MatrixXd CoM_traj; //QP N X 3 Matrix, col(1) = com X, col(2) = com Y, col(3) = com Z
    
} QP;


class CKubot {
public:

    CKubot();
    CKubot(const CKubot& orig);
    virtual ~CKubot();


    //***Variables***//
    int steps_msgs;
    double FBSize_msgs;
    double LRSize_msgs;
    double TurnSize_msgs;
    double footHeight_msgs;
    int stop_msgs;




    //* Control Mode and Command Flag

    unsigned int ControlMode;
    unsigned int CommandFlag;

    int joint_DoF; //Joint Degrees of Freedom


    //* enum

    enum JointNumber { // Kubot
        LHY = 0,
        LHR, LHP, LKN, LAP, LAR,
        RHY, RHR, RHP, RKN, RAP, RAR,
        LSP, LER, LHA, RSP, RER, RHA, NYA, HEP
    };

    enum EndPointAxis { // RoK-3's End point pos, ori axis
        X = 0, Y, Z, Roll, Pitch, Yaw
    };






    // int test = 10;
    int sign(double a);
    Eigen::Matrix3d transposeMat(Matrix3d mat);

    MatrixXd rotMatX(double q);
    MatrixXd rotMatY(double q);
    MatrixXd rotMatZ(double q);

    MatrixXd getTransformI0();
    MatrixXd getTransform6E();
    MatrixXd jointToTransform01(VectorXd q);
    MatrixXd jointToTransform01_R(VectorXd q);
    MatrixXd jointToTransform12(VectorXd q);
    MatrixXd jointToTransform23(VectorXd q);
    MatrixXd jointToTransform34(VectorXd q);
    MatrixXd jointToTransform45(VectorXd q);
    MatrixXd jointToTransform56(VectorXd q);

    MatrixXd jointToPosition(VectorXd q);
    MatrixXd jointToPosition(VectorXd q, double X, double Y, double Z);
    MatrixXd jointToPosition_R(VectorXd q);
    Matrix3d jointToRotMat(VectorXd q);
    Matrix3d jointToRotMat_R(VectorXd q);
    VectorXd rotToEuler(MatrixXd rotMat);

    MatrixXd jointToPosJac(VectorXd q);
    MatrixXd jointToPosJac_R(VectorXd q);
    MatrixXd jointToRotJac(VectorXd q);
    MatrixXd jointToRotJac_R(VectorXd q);

    MatrixXd pseudoInverseMat(MatrixXd A, double lambda);
    VectorXd rotMatToRotVec(MatrixXd C);
    MatrixXd angleAxisToRotMat(VectorXd rotVec);
    MatrixXd jointToGeoJac(VectorXd q);
    MatrixXd jointToGeoJac_R(VectorXd q);
    VectorXd inverseKinematics(Vector3d r_des, Matrix3d C_des, VectorXd q0, double tol);
    VectorXd inverseKinematics_R(Vector3d r_des, Matrix3d C_des, VectorXd q0, double tol);
    VectorXd Geometric_IK_L(VectorXd GB_cfg, VectorXd GF_cfg);
    VectorXd Geometric_IK_R(VectorXd GB_cfg, VectorXd GF_cfg);

    // ===================================================================
    //  kubot26 다리 수치 역기구학 (Damped Least Squares)
    //
    //  kubot26 의 힙은 구면관절이 아니다 (pitch 축과 roll 축이 75.6mm 떨어져
    //  있고 roll∩yaw 만 교차). 따라서 kubot25 에서 쓰던 Kajita 형 닫힌 해
    //  (힙을 한 점으로 가정) 를 쓸 수 없어 수치해로 대체한다.
    //
    //  q 인덱스 규약은 JointNumber enum 과 동일하게 유지한다:
    //    q(0)=Hip_yaw, q(1)=Hip_roll, q(2)=Hip_pitch,
    //    q(3)=Knee_pitch, q(4)=Ankle_pitch, q(5)=Ankle_roll
    //  (실제 링크 연결 순서는 pitch -> roll -> yaw -> knee -> ankle_p -> ankle_r)
    // ===================================================================
    enum LegSide { LEG_L = 0, LEG_R = 1 };

    MatrixXd legTransform(int side, VectorXd q);                 // base_link -> Foot (4x4)
    MatrixXd legGeoJacobian(int side, VectorXd q);               // base 좌표 기하 야코비안 (6x6)
    VectorXd legIK_fromGlobal(int side, VectorXd GB_PosOri, VectorXd GF_PosOri);
    VectorXd legIK(int side, MatrixXd T_des, VectorXd q_seed,
                   int maxIter = 60, double tol = 1e-6);         // DLS 역기구학
    void     clampLegJointLimits(int side, VectorXd &q);         // URDF 관절 한계로 포화
    double   lastIKError(int side) const { return IK_lastErr[side]; }
    int      lastIKIters(int side) const { return IK_lastIters[side]; }

    VectorXd IK_qPrev[2];      // warm-start 용 직전 해
    bool     IK_warmStarted[2] = {false, false};
    double   IK_lastErr[2]   = {0.0, 0.0};
    int      IK_lastIters[2] = {0, 0};
    double   IK_lambda = 0.05; // DLS 감쇠 계수

    //***Functions***//
    void abc();
    void FK(VectorXd zmp_fk);
    void IK();
    MatrixXd ZMPFK(VectorXd q);
    MatrixXd ZMPFK(VectorXd q, VectorXd qq);
    MatrixXd jointToTransform01_zmp(VectorXd q);

    double realTime, startTime;
    //////////save file////////////
    void save_file(int col_cnt);
    double save_jointData[25][60000];
    double save_previewData[11][60000];
    double save_footData[13][60000];

    // VectorXd init_pose(int param_1, VectorXd present, VectorXd real);

    bool Move_current;

    bool SWAY_READY;
    bool SWAY_ING;

    bool KICK_READY;
    bool KICK_ING;

    bool WALK_READY;
    bool WALK_ING;

    bool STANDUP_READY;
    bool STANDUP_ING;

    bool MOTION_READY;
    bool MOTION_ING;
    
    bool motion_1 = false;
    bool motion_2 = true;
    bool motion_3 = true;
    bool motion_4 = true;
    bool motion_5 = true;
    bool motion_6 = true;
    bool motion_7 = true;
    bool motion_8 = true;
    bool motion_9 = true;
    bool motion_10 = true;
    bool motion_11 = true;
    bool motion_12 = true;
    bool motion_13 = true;
    bool motion_14 = true;




    VectorXd actual_radian_vector;
    float refAngle[20];
    float kickAngle[20]; 
    float walkReadyAngle[20];
    float standupAngle[20];
    float standupRefAngle[20];
    int updown_cnt = 0;
    int DSP_count = 0;


    void HomePose(float return_time);
    void walkingReady(float readytime);
    void UpDown(float readytime);
    double cosWave(double amp, double period, double time, double int_pos);

    void setWalkingReadyPos(double init_x, double init_y, double init_z);
    // bool wave_flag;
    
    void sway();
    void DSP_baseRef();
    // zmp function and variable
    void initializeBkubot();
    void setZmpPreview(double previewTime); //* set about ZMP preview control
    void setZmpPreviewTime(double previewTime); //* set about ZMP preview time

    void initializeQP(void);

    void walkingPatternGenerator(int controlMODE, int CommandFLAG, double steps, double FBsize, double LRsize, double Turnsize, double footHeight,int stop); //* Planning Foot X,Y,Z, roll, pitch, yaw Reference for Walking
    void zmpPreviewControl(void); //* ZMP preview Control

    VectorXd local2Global(Vector3d Vector, VectorXd local);


    MatrixXd ZMP_DARE(MatrixXd& A, MatrixXd& B, MatrixXd& Q, MatrixXd& R);
    Eigen::EigenSolver<MatrixXd> eZMPSolver;


    BASE Base; //* coordinate of Body
    ZMP zmp; //* About ZMP, ZMP preview control, ZMP Observer and Controller
    BodyKine body_kine;
    WALK walking; //* Variable of Walking
    QP qp;


    ID systemID; //* System ID about the robot
//////////////////////////////////////////05.20
    void setWalkingStep(int steps, int Startfoot, double FBsize, double LRsize, double Turnsize, double footHeight);
    void setWalkingTime(double periodTime, double DSPratio);
    void FootStepsPlanner(); // turn관련 식만 주석처리하고 사용
    void generateZmpTraj(MatrixXd LfootPrints, MatrixXd RfootPrints);
    void initializeGlobalStepPosition();
    void lastGlobalStepPositionUpdate();
    void generateFootTraj(MatrixXd LfootPrints, MatrixXd RfootPrints);
    void globalPositionUpdate(VectorXd preLfoot, VectorXd preRfoot);

    Vector3d genreate_8th_BezierCurve(Vector3d point_0_1_2, Vector3d point_3, Vector3d point_4, Vector3d point_5, Vector3d point_6_7_8, double period, double time);
    int factorial(int n);
    
    void changeWalkingStep(double FBsize, double LRsize, double Turnsize);
    void FootPlannerWindowMove();

    void upperbodytest(float readytime);
    void standupFront(float readytime);
    void standupBack(float readytime);

    void hurdlemotion(float readytime);
    void BallCatchMotion(float readytime);
    void BallSwingMotion(float readytime);
    
    void WALKTHREETIMES(float readytime);
    
    void RAISE_LEFTARM(float readytime);
    
    void kickreadypose(float readytime);
    
    void GOALKEEPER(float readytime);
    void walkingphysics(float readytime);
    
    void hurdle(float hurdletime);
    
    
    void KickMotion_L(float readytime);
    void KickMotion_R(float readytime);
    void ACT_STANDUP_FRONT(float readytime);
    void ACT_STANDUP_BACK(float readytime);
    void TURN_L(float readytime);
    void TURN_R(float readytime);

    void KubotMode(const std_msgs::msg::Int32::SharedPtr msg);
    void Kubot_control_callback(const kubot26_pkgs::msg::KubotControlMsgs::SharedPtr msg);
    void Kubot_joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

    void FootPlannerWindowMove(const double step_y_offset);
    void FootStepsPlanner(MatrixXd LfootPrints, MatrixXd RfootPrints,
                          const double step_y_offset); // Planning Foot steps
                          
    ENDPOINT base, LFoot, RFoot; // base of robot, Both feet of robot
    bool joy_flag = false;
    double joy_FB_ref;
    int joy_cnt = 1;


    //* Functions related to Walking, are in CRobot_Walking.cpp

private:

    //***Variables***//    
    double init_t; //* Time for Initialize
    


    //***Functions***//

};
#endif
