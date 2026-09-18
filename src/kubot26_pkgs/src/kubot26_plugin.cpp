/*
Kubot package simulation code
kudos edu.ver
26.07.01 - Fixed for ROS2 Humble & Gazebo Classic
*/

//* Header file for C++
#include <stdio.h>
#include <iostream>
#include <time.h>
#include <functional> // std::bind 사용을 위해 필수 포함

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>

#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "kubot26_pkgs/msg/kubot_control_msgs.hpp"

#include <ignition/math/Vector3.hh>
#include <Eigen/Dense>

// KUDOS
#include "kudos/CKubot.h"

//* Print color
#define C_BLACK   "\033[30m"
#define C_RED     "\x1b[91m"
#define C_GREEN   "\x1b[92m"
#define C_YELLOW  "\x1b[93m"
#define C_BLUE    "\x1b[94m"
#define C_MAGENTA "\x1b[95m"
#define C_CYAN    "\x1b[96m"
#define C_RESET   "\x1b[0m"

using namespace std;

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::MatrixXf;
using Eigen::VectorXf;
using Eigen::Matrix2f;
using Eigen::Matrix3f;
using Eigen::Matrix4f;
using Eigen::Vector2f;
using Eigen::Vector3f;
using Eigen::Vector4f;

VectorXd test_vector(20);

namespace gazebo {

    class kubot26_plugin : public ModelPlugin
    {
        gazebo::common::Time last_update_time;
        gazebo::event::ConnectionPtr update_connection;
        double dt;
        double time = 0;

        physics::ModelPtr model; 

        physics::JointPtr L_Hip_yaw_joint;
        physics::JointPtr L_Hip_roll_joint;
        physics::JointPtr L_Hip_pitch_joint;
        physics::JointPtr L_Knee_pitch_joint;
        physics::JointPtr L_Ankle_pitch_joint;
        physics::JointPtr L_Ankle_roll_joint;

        physics::JointPtr R_Hip_yaw_joint;
        physics::JointPtr R_Hip_roll_joint;
        physics::JointPtr R_Hip_pitch_joint;
        physics::JointPtr R_Knee_pitch_joint;
        physics::JointPtr R_Ankle_pitch_joint;
        physics::JointPtr R_Ankle_roll_joint;

        physics::JointPtr L_Shoulder_roll_joint;
        physics::JointPtr L_Elbow_pitch_joint;
        physics::JointPtr L_Hand_pitch_joint;

        physics::JointPtr R_Shoulder_roll_joint;
        physics::JointPtr R_Elbow_pitch_joint;
        physics::JointPtr R_Hand_pitch_joint;

        physics::JointPtr Neck_yaw_joint;
        physics::JointPtr Head_pitch_joint;

        gazebo_ros::Node::SharedPtr node_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr KubotModesp;
        int ControlMode_by_ROS = 1;

        int StartFootCheck;
        bool FSW_STOP;
        
        std_msgs::msg::Float64 m_ZMP_x;
        std_msgs::msg::Float64 m_ZMP_y;
        std_msgs::msg::Float64 m_ZMPcontrol_x;
        std_msgs::msg::Float64 m_ZMPcontrol_y;
        std_msgs::msg::Float64 m_ZMP_x_l_margin;
        std_msgs::msg::Float64 m_ZMP_x_u_margin;
        std_msgs::msg::Float64 m_ZMP_y_l_margin;
        std_msgs::msg::Float64 m_ZMP_y_u_margin;

        std_msgs::msg::Float64 m_Base_refpos_x;
        std_msgs::msg::Float64 m_Base_refpos_y;
        std_msgs::msg::Float64 m_Base_refpos_z;
        
        std_msgs::msg::Float64 m_preview_ref_ZMP_x;
        std_msgs::msg::Float64 m_preview_ref_ZMP_y;
        std_msgs::msg::Float64 m_preview_COM_x;
        std_msgs::msg::Float64 m_preview_COM_y;

        std_msgs::msg::Float64 m_L_foot_ref_x;
        std_msgs::msg::Float64 m_L_foot_ref_z;
        std_msgs::msg::Float64 m_L_foot_FK_x;
        std_msgs::msg::Float64 m_L_foot_FK_z;

        std_msgs::msg::Float64 m_R_foot_ref_x;
        std_msgs::msg::Float64 m_R_foot_ref_z;
        std_msgs::msg::Float64 m_R_foot_FK_x;
        std_msgs::msg::Float64 m_R_foot_FK_z;

        std_msgs::msg::Float64 m_preview_FK_x;
        std_msgs::msg::Float64 m_preview_FK_y; 

        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr KubotModesp_pub;

        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr COM_x;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr COM_y;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr COM_z;

        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr P_ZMP_x;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr P_ZMP_y;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr P_ZMPcontrol_x;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr P_ZMPcontrol_y;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr P_preview_ref_ZMP_x;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr P_preview_ref_ZMP_y;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr P_preview_COM_x;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr P_preview_COM_y;

        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr P_preview_FK_x;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr P_preview_FK_y;       

        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr P_L_foot_ref_x;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr P_L_foot_ref_z;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr P_L_foot_FK_x;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr P_L_foot_FK_z;

        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr P_R_foot_ref_x;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr P_R_foot_ref_z;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr P_R_foot_FK_x;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr P_R_foot_FK_z;
        
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr P_Base_refpos_x;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr P_Base_refpos_y;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr P_Base_refpos_z;

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr P_joint_states;

        std_msgs::msg::Float64 KubotModesp_msg;     

        rclcpp::Subscription<kubot26_pkgs::msg::KubotControlMsgs>::SharedPtr Kubot_control_mode_sub;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr Kubot_joy_sub;

        enum
        { 
            LHY = 0, LHR, LHP, LKN, LAP, LAR, RHY, RHR, RHP, RKN, RAP, RAR, LSP, LER, LHA, RSP, RER, RHA, NYA, HEP
        };

        int nDoF; 

        typedef struct RobotJoint 
        {
            double targetDegree; 
            double targetRadian; 
            double init_targetradian;
            double targetRadian_interpolation; 
            double targetVelocity; 
            double targetTorque; 
            double actualDegree; 
            double actualRadian; 
            double actualVelocity; 
            double actualRPM; 
            double actualTorque; 
            double Kp;
            double Ki;
            double Kd;
            double torqueLimit;  // 장착 모터의 스톨 토크 [Nm]
        } ROBO_JOINT;
        ROBO_JOINT* joint;       

        bool joint_by_algorithm_update = false;
         
        sensors::SensorPtr Sensor;
        sensors::ImuSensorPtr IMU; 

        CKubot Kubot; 

    public :
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override; 
        
        // 중요: 가제보 이벤트 전달 포맷에 호환되도록 시그니처 변경 (UpdateInfo 인자 매핑 구조 대응)
        void UpdateAlgorithm(const common::UpdateInfo & _info); 

        void jointUpdateByAlgorithm(); 
        void setjoints();     
        void getjointdata(); 
        void setsensor();
        void getsensordata();
        void jointcontroller();
        void initializejoint(); 
        void setjointPIDgain(); 

        void KubotMode(const std_msgs::msg::Int32::SharedPtr msg);
        void Kubot_control_callback(const kubot26_pkgs::msg::KubotControlMsgs::SharedPtr msg);
        void Kubot_joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msgs);
    };
    GZ_REGISTER_MODEL_PLUGIN(kubot26_plugin)
}

void gazebo::kubot26_plugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    // 가제보 표준 출력 대신 강제 콘솔 출력으로 진입 여부 파악
    std::cout << "========================================" << std::endl;
    std::cout << "   KUBOT26 MODEL PLUGIN STARTING...     " << std::endl;
    std::cout << "========================================" << std::endl;

    // 1. 모델 포인터 및 월드 포인터 저장
    this->model = _model;

    // 2. 중요: 기존의 Node 생성 방식을 완전히 지우고 gazebo_ros 인터페이스 연결
    // rclcpp::Node::make_shared를 가제보 내부에서 호출하면 스레드가 충돌하여 터집니다.
    // 여러 대를 동시에 띄우면 플러그인 이름이 그대로 ROS 노드 이름이 되어 충돌한다.
    // (이름이 겹치면 Node::Get 은 nullptr 를 돌려주고, 그대로 쓰면 세그폴트로 죽는다)
    // 모델 이름으로 고유화한다.
    this->node_ = gazebo_ros::Node::Get(_sdf, this->model->GetName() + "_plugin");
    if (!this->node_) {
        gzerr << "[KUBOT] ROS 노드 생성 실패 (이름 충돌 가능): "
              << this->model->GetName() << std::endl;
        return;
    }

    // 3. ROS2 토픽 구독(Subscription) 설정
    this->KubotModesp = this->node_->create_subscription<std_msgs::msg::Int32>(
        "KubotMode", 10, std::bind(&kubot26_plugin::KubotMode, this, std::placeholders::_1));
    this->Kubot_control_mode_sub = this->node_->create_subscription<kubot26_pkgs::msg::KubotControlMsgs>(
        "Kubot_Control_Msg", 10, std::bind(&kubot26_plugin::Kubot_control_callback, this, std::placeholders::_1));
    this->Kubot_joy_sub = this->node_->create_subscription<sensor_msgs::msg::Joy>( 
        "joy", 10, std::bind(&kubot26_plugin::Kubot_joy_callback, this, std::placeholders::_1));
    
    // 4. 로봇 내부 링크 및 조인트 포인터 매핑
    setjoints();
    setsensor();

    nDoF = 20; 
    joint = new ROBO_JOINT[nDoF](); 
    
    initializejoint();
    setjointPIDgain();

    // 5. 로봇 알고리즘 초기화
    Kubot.initializeBkubot();
    Kubot.setWalkingReadyPos(0, 0, 0.38);

    // 6. ROS2 퍼블리셔(Publisher) 설정 (기존 항목 유지)
    KubotModesp_pub = this->node_->create_publisher<std_msgs::msg::Float64>("command/KubotMode", 10);
    P_preview_COM_x = this->node_->create_publisher<std_msgs::msg::Float64>("COM_x", 10);
    P_preview_ref_ZMP_x = this->node_->create_publisher<std_msgs::msg::Float64>("zmp_ref_x", 10);
    P_preview_COM_y = this->node_->create_publisher<std_msgs::msg::Float64>("COM_y", 10);
    P_preview_ref_ZMP_y = this->node_->create_publisher<std_msgs::msg::Float64>("zmp_ref_y", 10);
    P_ZMP_x = this->node_->create_publisher<std_msgs::msg::Float64>("zmp_X", 10);
    P_ZMP_y = this->node_->create_publisher<std_msgs::msg::Float64>("zmp_Y", 10);
    P_preview_FK_x = this->node_->create_publisher<std_msgs::msg::Float64>("zmpFK_X", 10);
    P_preview_FK_y = this->node_->create_publisher<std_msgs::msg::Float64>("zmpFK_Y", 10);
    P_L_foot_ref_x = this->node_->create_publisher<std_msgs::msg::Float64>("L_foot_ref_X", 10);
    P_L_foot_ref_z = this->node_->create_publisher<std_msgs::msg::Float64>("L_foot_ref_Z", 10);
    P_L_foot_FK_x = this->node_->create_publisher<std_msgs::msg::Float64>("L_foot_FK_X", 10);
    P_L_foot_FK_z = this->node_->create_publisher<std_msgs::msg::Float64>("L_foot_FK_Z", 10);
    P_R_foot_ref_x = this->node_->create_publisher<std_msgs::msg::Float64>("R_foot_ref_X", 10);
    P_R_foot_ref_z = this->node_->create_publisher<std_msgs::msg::Float64>("R_foot_ref_Z", 10);
    P_R_foot_FK_x = this->node_->create_publisher<std_msgs::msg::Float64>("R_foot_FK_X", 10);
    P_R_foot_FK_z = this->node_->create_publisher<std_msgs::msg::Float64>("R_foot_FK_Z", 10);
    P_joint_states = this->node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    // 7. 타이머 초기화 및 이벤트 연결
    this->last_update_time = this->model->GetWorld()->SimTime();

    std::cout << "===== BEFORE ConnectWorldUpdateBegin =====" << std::endl;

    // [컴파일 에러 해결]: boost::bind에 C++ 글로벌 플레이스홀더인 ::_1 을 명시적으로 던져주어
    // Arguments 선언 에러를 완벽하게 고치고 가제보 이벤트 엔진에 고정합니다.
    this->update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(
        boost::bind(&kubot26_plugin::UpdateAlgorithm, this, ::_1));

    std::cout << "========================================" << std::endl;
    std::cout << "   KUBOT26 MODEL PLUGIN LOADED SUCCESS! " << std::endl;
    std::cout << "   JOINT STATE PUBLISHER ONLINE!        " << std::endl;
    std::cout << "========================================" << std::endl;
} // Load 함수 끝

void gazebo::kubot26_plugin::UpdateAlgorithm(const common::UpdateInfo & _info)
{
    common::Time current_time = model->GetWorld()->SimTime();
    dt = current_time.Double() - last_update_time.Double();
    
    if (dt <= 0.0) {
        // 시뮬레이션 리셋(/reset_simulation, Gazebo 의 Reset World)으로 sim time 이
        // 0 으로 되감기면 dt 가 음수가 된다. 여기서 기준 시각을 다시 맞춰주지 않으면
        // 매 틱 이 가드에 걸려 제어 루프가 영영 멈추고, 토크가 0 이 되어 로봇이
        // 힘없이 주저앉는다.
        last_update_time = current_time;

        if (dt < 0.0) {
            // 되감김 = 리셋. 내부 시계와 모션 타이머도 같이 맞춘다.
            time = 0.0;
            Kubot.realTime = 0.0;
            Kubot.startTime = 0.0;
        }
        return;
    }

    // 깔끔하게 삭제 완료! 가제보 고유 익스큐터가 알아서 스핀을 처리합니다.

    time = time + dt;
    Kubot.realTime =  Kubot.realTime + dt;
    last_update_time = current_time;
        
    getjointdata();
    getsensordata();

    static double steps    = 0;
    static double FBSize   = 0;
    static double LRSize   = 0;
    static double TurnSize = 0;

    double zmpFK_X = 0;
    double zmpFK_Y = 0;
    double zmpFK_Z = 0;

    double L_foot_ref_x = 0;
    double L_foot_ref_y = 0;
    double L_foot_ref_z = 0;
    double R_foot_ref_x = 0;
    double R_foot_ref_y = 0;
    double R_foot_ref_z = 0;

    double L_foot_FK_x = 0;
    double L_foot_FK_y = 0;
    double L_foot_FK_z = 0;
    double R_foot_FK_x = 0;
    double R_foot_FK_y = 0;   
    double R_foot_FK_z = 0;

    VectorXd L_foot_FKcheck(6);
    VectorXd R_foot_FKcheck(6);

    static int con_count = 0;

    // [중요 고침 부문]: tasktime 주기마다 알고리즘 제어문을 실행하도록 주기 분기 제어
    if (con_count % (int) (tasktime * 1000 + 0.001) == 0)
    {
        // =================================================================
        // [ROS1 원본 시퀀스 논리 흐름 완벽 복원]
        // rqt 토픽으로 가제보 스레드에 수신된 ControlMode_by_ROS를 기준으로 제어 플래그를 전환합니다.
        // =================================================================
        switch (ControlMode_by_ROS) {

            case 1: // HOME POSE
                Kubot.startTime = Kubot.realTime;
                Kubot.ControlMode = CTRLMODE_HOMEPOSE;
                Kubot.CommandFlag = RETURN_HOMEPOSE;
                printf("[KUBOT SUCCESS] 1번 수신: Kubot home pose! \n");
                Kubot.Move_current = false;
                ControlMode_by_ROS = 0; // 중복 진입 방지 리셋
                break;

            case 2: // WALK READY POSE
                Kubot.startTime = Kubot.realTime;
                Kubot.ControlMode = CTRLMODE_WALKREADY;
                Kubot.CommandFlag = GOTO_WALK_READY_POS;
                printf("[KUBOT SUCCESS] 2번 수신: Kubot walk ready pose! \n");
                Kubot.Move_current = false;
                ControlMode_by_ROS = 0;
                break;

            case 3: // RAISE LEFT ARM
                Kubot.startTime = Kubot.realTime;
                Kubot.ControlMode = CTRLMODE_RAISELEFTARM;
                Kubot.CommandFlag = Raise_Left_Arm;
                printf("[KUBOT SUCCESS] 3번 수신: Raise_Left_Arm! \n");
                Kubot.Move_current = false;
                ControlMode_by_ROS = 0;
                break;

            case 4: // WALK START (3보 전진 보행)
                Kubot.startTime = Kubot.realTime;
                Kubot.ControlMode = CTRLMODE_WALKWALKWALK;
                Kubot.CommandFlag = WALKTHREETIMES; 
                printf("[KUBOT SUCCESS] 4번 수신: Kubot walking start! \n");
                ControlMode_by_ROS = 0;
                break;

            case 5: // KICK READY POSE
                Kubot.startTime = Kubot.realTime;
                Kubot.ControlMode = CTRLMODE_KICKREADYPOSE;
                Kubot.CommandFlag = kickreadypose;
                printf("[KUBOT SUCCESS] 5번 수신: kickreadypose! \n");
                Kubot.Move_current = false;
                ControlMode_by_ROS = 0;
                break;

            case 7: // WALKING PHYSICS MODE
                Kubot.startTime = Kubot.realTime;
                Kubot.ControlMode = CTRLMODE_PHYSICSMODE;
                Kubot.CommandFlag = walkingphysics; 
                printf("[KUBOT SUCCESS] 7번 수신: walkingphysics! \n");
                Kubot.Move_current = false;
                ControlMode_by_ROS = 0;
                break;

            case 25: // STANDUP FRONT
                Kubot.motion_1 = false;
                Kubot.startTime = Kubot.realTime;
                Kubot.ControlMode = CTRLMODE_STANDUP_TEST;
                Kubot.CommandFlag = ACT_STANDUP_FRONT;
                printf("[KUBOT SUCCESS] 25번 수신: STANDUP FRONT MOVE START! \n");
                Kubot.Move_current = false;
                ControlMode_by_ROS = 0;
                Kubot.STANDUP_READY = true;
                Kubot.STANDUP_ING = false;
                break;

            case 26: // STANDUP BACK
                Kubot.motion_1 = false;
                Kubot.startTime = Kubot.realTime;
                Kubot.ControlMode = CTRLMODE_STANDUP_TEST;
                Kubot.CommandFlag = ACT_STANDUP_BACK;
                printf("[KUBOT SUCCESS] 26번 수신: STANDUP BACK MOVE START! \n");
                Kubot.Move_current = false;
                ControlMode_by_ROS = 0;
                Kubot.STANDUP_READY = true;
                Kubot.STANDUP_ING = false;
                break;

            case 44: // KICK LEFT
                Kubot.motion_1 = false;
                Kubot.motion_2 = true;
                Kubot.motion_3 = true;
                Kubot.motion_4 = true;
                Kubot.motion_5 = true;
                Kubot.motion_6 = true;

                Kubot.startTime = Kubot.realTime;
                Kubot.ControlMode = CTRLMODE_MOTION;
                Kubot.CommandFlag = ACT_KICK_L;
                printf("[KUBOT SUCCESS] 44번 수신: KICK THE BALL (LEFT)! \n");
                Kubot.Move_current = false;
                ControlMode_by_ROS = 0;
                Kubot.MOTION_READY = true;
                Kubot.MOTION_ING = false;
                break;

            case 45: // KICK RIGHT
                Kubot.startTime = Kubot.realTime;
                Kubot.ControlMode = CTRLMODE_MOTION;
                Kubot.CommandFlag = ACT_KICK_R;
                printf("[KUBOT SUCCESS] 45번 수신: KICK MOTION R START! \n");
                Kubot.Move_current = false;
                ControlMode_by_ROS = 0;
                Kubot.MOTION_READY = true;
                Kubot.MOTION_ING = false;
                break;

            case 70: // TURN LEFT
                Kubot.startTime = Kubot.realTime;
                Kubot.ControlMode = CTRLMODE_TURNING_L;
                Kubot.CommandFlag = ACT_TURNING_L;
                printf("[KUBOT SUCCESS] 70번 수신: TURN_L START! \n");
                Kubot.Move_current = false;
                ControlMode_by_ROS = 0;
                Kubot.MOTION_READY = true;
                Kubot.MOTION_ING = false;
                break;

            case 71: // TURN RIGHT
                Kubot.startTime = Kubot.realTime;
                Kubot.ControlMode = CTRLMODE_TURNING_R;
                Kubot.CommandFlag = ACT_TURNING_R;
                printf("[KUBOT SUCCESS] 71번 수신: TURN_R START! \n");
                Kubot.Move_current = false;
                ControlMode_by_ROS = 0;
                Kubot.MOTION_READY = true;
                Kubot.MOTION_ING = false;
                break;

            case 100: // GOALKEEPER MODE
                Kubot.startTime = Kubot.realTime;
                Kubot.ControlMode = CTRLMODE_GOALKEEPERMODE;
                Kubot.CommandFlag = GOALKEEPER;
                printf("[KUBOT SUCCESS] 100번 수신: GOALKEEPER! \n");
                Kubot.Move_current = false;
                ControlMode_by_ROS = 0;
                break;

            case 99: // SAVE FILE LOG
                printf("[KUBOT SUCCESS] 99번 수신: save file START! \n");
                Kubot.save_file(50000);
                printf("[KUBOT SUCCESS] save file DONE! \n");
                ControlMode_by_ROS = 0;
                break;

            default:
                break;
        }
        // =================================================================
        switch (Kubot.CommandFlag) 
        {
            case GOTO_WALK_READY_POS: 
                if (Kubot.Move_current == false) {
                    Kubot.walkingReady(5.0);
                    for (int j = 0; j < nDoF; j++) joint[j].targetRadian = Kubot.refAngle[j];
                } else {
                    Kubot.CommandFlag = NONE_ACT;
                    printf("WALK_READY COMPLETE !\n");
                }
                break;

            case RETURN_HOMEPOSE: 
                if (Kubot.Move_current == false) {
                    Kubot.HomePose(5.0);
                    for (int j = 0; j < nDoF; j++) joint[j].targetRadian = Kubot.refAngle[j];
                } else {
                    printf("RETURN_HOMEPOSE COMPLETE !\n");
                    Kubot.ControlMode = CTRLMODE_HOMEPOSE;
                    Kubot.CommandFlag = NONE_ACT;
                }
                break;

            case Raise_Left_Arm:
                if (Kubot.Move_current == false){
                    Kubot.RAISE_LEFTARM(2.5);
                    for (int j = 0; j < nDoF; j++) joint[j].targetRadian = Kubot.refAngle[j];
                } else {
                    printf("RAISE_LEFT_ARM COMPLETE !\n");
                    Kubot.CommandFlag = NONE_ACT;
                }
                break;

            case WALKTHREETIMES:
                if (Kubot.Move_current == false){
                    Kubot.WALKTHREETIMES(2.5);
                    for (int j = 0; j < nDoF; j++) joint[j].targetRadian = Kubot.refAngle[j];
                } else {
                    printf("WALKWALKWALK COMPLETE !\n");
                    Kubot.CommandFlag = NONE_ACT;
                }
                break;
            
            case kickreadypose:
                if (Kubot.Move_current == false){
                    Kubot.kickreadypose(2.5);
                    for (int j = 0; j < nDoF; j++) joint[j].targetRadian = Kubot.refAngle[j];
                } else {
                    printf("KICKREADYPOSE COMPLETE !\n");
                    Kubot.CommandFlag = NONE_ACT;
                }
                break;    

            case GOALKEEPER:
                if (Kubot.Move_current == false){
                    Kubot.GOALKEEPER(2.5);
                    for (int j = 0; j < nDoF; j++) joint[j].targetRadian = Kubot.refAngle[j];
                } else {
                    printf("GOALKEEPER COMPLETE !\n");
                    Kubot.CommandFlag = NONE_ACT;
                }
                break;    
            
            case walkingphysics:
                if (Kubot.Move_current == false){
                    Kubot.walkingphysics(2.5);
                    for (int j = 0; j < nDoF; j++) joint[j].targetRadian = Kubot.refAngle[j];
                } else {
                    printf("walkingphysics COMPLETE !\n");
                    Kubot.CommandFlag = NONE_ACT;
                }
                break;    

            case ACT_KICK_L:
                if (Kubot.Move_current == false){
                    Kubot.KickMotion_L(2.5);
                    for (int j = 0; j < nDoF; j++) joint[j].targetRadian = Kubot.refAngle[j];
                } else {
                    printf("KICKING COMPLETE !\n");
                    Kubot.CommandFlag = NONE_ACT;
                }
                break;

            case ACT_KICK_R:
                if (Kubot.Move_current == false){
                    Kubot.KickMotion_R(2.5);
                    for (int j = 0; j < nDoF; j++) joint[j].targetRadian = Kubot.refAngle[j];
                } else {
                    printf("KICKING COMPLETE !\n");
                    Kubot.CommandFlag = NONE_ACT;
                }
                break;
            
            case ACT_TURNING_L:
                if (Kubot.MOTION_ING == false) {
                    Kubot.TURN_L(2.5);
                    for (int j = 0; j < nDoF; j++) joint[j].targetRadian = Kubot.refAngle[j];
                } else {
                    printf("=====TURN LEFT MOTION COMPLETE !=====\n");
                }
                break;

            case ACT_TURNING_R:
                if (Kubot.MOTION_ING == false) {
                    Kubot.TURN_R(2.5);
                    for (int j = 0; j < nDoF; j++) joint[j].targetRadian = Kubot.refAngle[j];
                } else {
                    printf("=====TURN RIGHT MOTION COMPLETE !=====\n");
                    Kubot.ControlMode = CTRLMODE_WALKING_TEST;
                    Kubot.CommandFlag = ACT_TURNING_R;
                    Kubot.zmp.previewControl.count = 0;
                    Kubot.stop_msgs = 0;
                    Kubot.Move_current = true;
                    ControlMode_by_ROS = 0;
                    Kubot.WALK_READY = true;
                    Kubot.WALK_ING = false;
                }
                break; 

            case ACT_STANDUP_FRONT: 
                if (Kubot.STANDUP_ING == false) {
                    Kubot.standupFront(3.0);
                } else {
                    Kubot.STANDUP_ING = false;
                    Kubot.ControlMode = CTRLMODE_WALKING_TEST;
                    Kubot.CommandFlag = ACT_START_WALK;
                    Kubot.zmp.previewControl.count = 0;
                    Kubot.stop_msgs = 0;
                    Kubot.Move_current = true;
                    ControlMode_by_ROS = 0;
                    Kubot.WALK_READY = true;
                    Kubot.WALK_ING = false;
                    printf("=====STANDUP FRONT COMPLETE !=====\n");
                }
                break;
                
            case ACT_STANDUP_BACK: 
                if (Kubot.STANDUP_ING == false) {
                    Kubot.standupBack(2.0);
                } else {
                    Kubot.STANDUP_ING = false;
                    Kubot.ControlMode = CTRLMODE_WALKING_TEST;
                    Kubot.CommandFlag = ACT_START_WALK;
                    Kubot.zmp.previewControl.count = 0;
                    Kubot.stop_msgs = 0;
                    Kubot.Move_current = true;
                    ControlMode_by_ROS = 0;
                    Kubot.WALK_READY = true;
                    Kubot.WALK_ING = false;
                    printf("=====STANDUP FRONT COMPLETE !=====\n");
                }
                break;

            case NONE_ACT:
                FBSize = 0; LRSize = 0; TurnSize = 0;
                break;
            default: break;
        }

        Kubot.walkingPatternGenerator(Kubot.ControlMode,Kubot.CommandFlag,Kubot.steps_msgs,Kubot.FBSize_msgs,Kubot.LRSize_msgs,Kubot.TurnSize_msgs,Kubot.footHeight_msgs,Kubot.stop_msgs);

        if (Kubot.ControlMode != CTRLMODE_WALKREADY) {
            VectorXd joint_IK(12);

            if (Kubot.Move_current == true && Kubot.CommandFlag == UPDOWN) {
                VectorXd GFL_Vector(6); GFL_Vector << 0, 0.05, 0, 0, 0, 0;
                VectorXd GFR_Vector(6); GFR_Vector << 0, -0.05, 0, 0, 0, 0;
                VectorXd GB_Vector(6);
                double ref_B_x = 0; double ref_B_y = 0; double ref_B_z; 
                double z_max = 0.42; double z_min = 0.34; double T = 1000;

                ref_B_z = Kubot.cosWave(z_max-z_min, T, Kubot.updown_cnt, z_min);
                Kubot.updown_cnt = Kubot.updown_cnt + 1;
                GB_Vector << ref_B_x, ref_B_y, ref_B_z, 0, 0, 0;
                
                joint_IK << Kubot.Geometric_IK_L(GB_Vector, GFL_Vector), Kubot.Geometric_IK_R(GB_Vector, GFR_Vector);
            }                   
            else if(Kubot.CommandFlag == SWAYMOTION && Kubot.SWAY_READY == true) {
                VectorXd GBS_Vector(6);
                VectorXd GFL_Vector(6); GFL_Vector << 0, 0.05, 0, 0, 0, 0;
                VectorXd GFR_Vector(6); GFR_Vector << 0, -0.05, 0, 0, 0, 0;

                GBS_Vector << Kubot.zmp.previewControl.X.CoM, Kubot.zmp.previewControl.Y.CoM, 0.34, 0, 0, 0;
                Kubot.zmp.previewControl.count = Kubot.zmp.previewControl.count + 1;
                joint_IK << Kubot.Geometric_IK_L(GBS_Vector, GFL_Vector), Kubot.Geometric_IK_R(GBS_Vector, GFR_Vector);
            }
            else if(Kubot.CommandFlag == KICKTEST && Kubot.KICK_READY == true) {
                VectorXd GBS_Vector(6); VectorXd GFL_Vector(6); VectorXd GFR_Vector(6);
                double ref_RF_x; double ref_RF_z;
                double RFx1_max = 0.05; double RFx1_min = 0.0;
                double RFx2_max = 0.1;  double RFx2_min = 0.05;
                double RFz_max = 0.06;  double RFz_min = 0.0;
                double T = 2000;

                GBS_Vector << Kubot.zmp.previewControl.X.CoM, Kubot.zmp.previewControl.Y.CoM, 0.38, 0, 0, 0;

                if (Kubot.zmp.previewControl.count < 3000) {
                    GFR_Vector << 0, -0.05, 0, 0, 0, 0; GFL_Vector << 0, 0.05, 0, 0, 0, 0;
                }
                else if (Kubot.zmp.previewControl.count >= 3000 && Kubot.zmp.previewControl.count < 5000) {
                    ref_RF_z = Kubot.cosWave(RFz_max-RFz_min, T, Kubot.zmp.previewControl.count-3000, RFz_min);
                    GFR_Vector << 0, -0.05, ref_RF_z, 0, 0, 0; GFL_Vector << 0, 0.05, 0, 0, 0, 0;
                }
                else if (Kubot.zmp.previewControl.count >= 5000 && Kubot.zmp.previewControl.count < 7000) {
                    ref_RF_x = Kubot.cosWave(RFx1_max-RFx1_min, T, Kubot.zmp.previewControl.count-5000, RFx1_min);
                    GFR_Vector << ref_RF_x, -0.05, RFz_max, 0, 0, 0; GFL_Vector << 0, 0.05, 0, 0, 0, 0;
                }
                else if (Kubot.zmp.previewControl.count >= 7000 && Kubot.zmp.previewControl.count < 9000) {
                    ref_RF_z = Kubot.cosWave(RFz_min-RFz_max, T, Kubot.zmp.previewControl.count-7000, RFz_max);
                    GFR_Vector << RFx1_max, -0.05, ref_RF_z, 0, 0, 0; GFL_Vector << 0, 0.05, 0, 0, 0, 0;
                }
                else if(Kubot.zmp.previewControl.count >= 9000 && Kubot.zmp.previewControl.count < 10000) {
                    GFR_Vector << 0.05, -0.05, 0, 0, 0, 0; GFL_Vector << 0, 0.05, 0, 0, 0, 0;
                }
                else if(Kubot.zmp.previewControl.count >= 10000 && Kubot.zmp.previewControl.count < 12000) {
                    ref_RF_z = Kubot.cosWave(RFz_max-RFz_min, T, Kubot.zmp.previewControl.count-10000, RFz_min);
                    GFR_Vector << 0.05, -0.05, 0, 0, 0, 0; GFL_Vector << 0, 0.05, ref_RF_z, 0, 0, 0;
                }
                else if (Kubot.zmp.previewControl.count >= 12000 && Kubot.zmp.previewControl.count < 14000) {
                    ref_RF_x = Kubot.cosWave(RFx1_max-RFx1_min, T, Kubot.zmp.previewControl.count-12000, RFx1_min);
                    GFR_Vector << 0.05, -0.05, 0, 0, 0, 0; GFL_Vector << ref_RF_x, 0.05, RFz_max, 0, 0, 0;
                }
                else if (Kubot.zmp.previewControl.count >= 14000 && Kubot.zmp.previewControl.count < 16000) {
                    ref_RF_z = Kubot.cosWave(RFz_min-RFz_max, T, Kubot.zmp.previewControl.count-14000, RFz_max);
                    GFR_Vector << 0.05, -0.05, 0, 0, 0, 0; GFL_Vector << RFx1_max, 0.05, ref_RF_z, 0, 0, 0;
                }
                else if (Kubot.zmp.previewControl.count >= 16000 && Kubot.zmp.previewControl.count < 17000) {
                    GFR_Vector << 0.05, -0.05, 0, 0, 0, 0; GFL_Vector << 0.05, 0.05, 0, 0, 0, 0;
                }
                else if (Kubot.zmp.previewControl.count >= 17000 && Kubot.zmp.previewControl.count < 19000) {
                    ref_RF_z = Kubot.cosWave(RFz_max-RFz_min, T, Kubot.zmp.previewControl.count-17000, RFz_min);
                    GFR_Vector << 0.05, -0.05, ref_RF_z, 0, 0, 0; GFL_Vector << 0.05, 0.05, 0, 0, 0, 0;
                }
                else if (Kubot.zmp.previewControl.count >= 19000 && Kubot.zmp.previewControl.count < 21000) {
                    ref_RF_x = Kubot.cosWave(RFx2_max-RFx2_min, T, Kubot.zmp.previewControl.count-19000, RFx2_min);
                    GFR_Vector << ref_RF_x, -0.05, RFz_max, 0, 0, 0; GFL_Vector << 0.05, 0.05, 0, 0, 0, 0;
                }
                else if (Kubot.zmp.previewControl.count >= 21000 && Kubot.zmp.previewControl.count < 23000) {
                    ref_RF_z = Kubot.cosWave(RFz_min-RFz_max, T, Kubot.zmp.previewControl.count-21000, RFz_max);
                    GFR_Vector << RFx2_max, -0.05, ref_RF_z, 0, 0, 0; GFL_Vector << 0.05, 0.05, 0, 0, 0, 0;
                }
                else if(Kubot.zmp.previewControl.count >= 23000 && Kubot.zmp.previewControl.count < 24000) {
                    GFR_Vector << 0.1, -0.05, 0, 0, 0, 0; GFL_Vector << 0.05, 0.05, 0, 0, 0, 0;
                }
                else if(Kubot.zmp.previewControl.count >= 24000 && Kubot.zmp.previewControl.count < 26000) {
                    ref_RF_z = Kubot.cosWave(RFz_max-RFz_min, T, Kubot.zmp.previewControl.count-24000, RFz_min);
                    GFR_Vector << 0.1, -0.05, 0, 0, 0, 0; GFL_Vector << 0.05, 0.05, ref_RF_z, 0, 0, 0;
                }
                else if (Kubot.zmp.previewControl.count >= 26000 && Kubot.zmp.previewControl.count < 28000) {
                    ref_RF_x = Kubot.cosWave(RFx2_max-RFx2_min, T, Kubot.zmp.previewControl.count-26000, RFx2_min);
                    GFR_Vector << 0.1, -0.05, 0, 0, 0, 0; GFL_Vector << ref_RF_x, 0.05, RFz_max, 0, 0, 0;
                }
                else if (Kubot.zmp.previewControl.count >= 28000 && Kubot.zmp.previewControl.count < 30000) {
                    ref_RF_z = Kubot.cosWave(RFz_min-RFz_max, T, Kubot.zmp.previewControl.count-28000, RFz_max);
                    GFR_Vector << 0.1, -0.05, 0, 0, 0, 0; GFL_Vector << RFx2_max, 0.05, ref_RF_z, 0, 0, 0;
                }
                else if(Kubot.zmp.previewControl.count >= 30000) {
                    GFR_Vector << 0.1, -0.05, 0, 0, 0, 0; GFL_Vector << 0.1, 0.05, 0, 0, 0, 0;
                }

                L_foot_ref_x = GFL_Vector(0); L_foot_ref_z = GFL_Vector(2);
                R_foot_ref_x = GFR_Vector(0); R_foot_ref_z = GFR_Vector(2);

                joint_IK << Kubot.Geometric_IK_L(GBS_Vector, GFL_Vector), Kubot.Geometric_IK_R(GBS_Vector, GFR_Vector);
                Kubot.zmp.previewControl.count = Kubot.zmp.previewControl.count + 1;

                L_foot_FKcheck << Kubot.jointToPosition(test_vector, GBS_Vector(0), GBS_Vector(1), GBS_Vector(2));
                L_foot_FK_x = L_foot_FKcheck(0); L_foot_FK_z = L_foot_FKcheck(2);
                R_foot_FK_x = L_foot_FKcheck(3); R_foot_FK_z = L_foot_FKcheck(5);

                VectorXd zmpFKcheck(3);
                zmpFKcheck << Kubot.ZMPFK(test_vector, L_foot_FKcheck);
                zmpFK_X = zmpFKcheck(0); zmpFK_Y = zmpFKcheck(1);
            }
            else if(Kubot.CommandFlag == ACT_TEST_WALK || Kubot.CommandFlag == ACT_INF_WALK || (Kubot.CommandFlag == ACT_STOP_WALK && Kubot.WALK_READY == true)) {
                VectorXd GBS_Vector(6); VectorXd GFL_Vector(6); VectorXd GFR_Vector(6);

                GBS_Vector << Kubot.zmp.previewControl.X.CoM, Kubot.zmp.previewControl.Y.CoM, 0.38, 0, 0, Kubot.Base.refpos(Kubot.Yaw);
                GFL_Vector << Kubot.LFoot.ref_G_pattern_pos(0), Kubot.LFoot.ref_G_pattern_pos(1), Kubot.LFoot.ref_G_pattern_pos(2), 0, 0, Kubot.LFoot.refpos(5);
                GFR_Vector << Kubot.RFoot.ref_G_pattern_pos(0), Kubot.RFoot.ref_G_pattern_pos(1), Kubot.RFoot.ref_G_pattern_pos(2), 0, 0, Kubot.RFoot.refpos(5);

                L_foot_ref_x = GFL_Vector(0); L_foot_ref_y = GFL_Vector(1); L_foot_ref_z = GFL_Vector(2);
                R_foot_ref_x = GFR_Vector(0); R_foot_ref_y = GFR_Vector(1); R_foot_ref_z = GFR_Vector(2);

                joint_IK << Kubot.Geometric_IK_L(GBS_Vector, GFL_Vector), Kubot.Geometric_IK_R(GBS_Vector, GFR_Vector);

                L_foot_FKcheck << Kubot.jointToPosition(test_vector, GBS_Vector(0), GBS_Vector(1), GBS_Vector(2));
                L_foot_FK_x = L_foot_FKcheck(0); L_foot_FK_y = L_foot_FKcheck(1); L_foot_FK_z = L_foot_FKcheck(2);
                R_foot_FK_x = L_foot_FKcheck(3); R_foot_FK_y = L_foot_FKcheck(4); R_foot_FK_z = L_foot_FKcheck(5);

                VectorXd zmpFKcheck(3);
                zmpFKcheck << Kubot.ZMPFK(test_vector, L_foot_FKcheck);
                zmpFK_X = zmpFKcheck(0); zmpFK_Y = zmpFKcheck(1); zmpFK_Z = zmpFKcheck(2);
            }

            if (Kubot.ControlMode == CTRLMODE_HOMEPOSE || Kubot.ControlMode == CTRLMODE_STANDUP_TEST || Kubot.ControlMode == CTRLMODE_SRCMOTION || Kubot.ControlMode == CTRLMODE_WALKREADY)
            {
                joint[LHY].targetRadian = Kubot.refAngle[0]; joint[LHR].targetRadian = Kubot.refAngle[1]; joint[LHP].targetRadian = Kubot.refAngle[2];
                joint[LKN].targetRadian = Kubot.refAngle[3]; joint[LAP].targetRadian = Kubot.refAngle[4]; joint[LAR].targetRadian = Kubot.refAngle[5];
                joint[RHY].targetRadian = Kubot.refAngle[6]; joint[RHR].targetRadian = Kubot.refAngle[7]; joint[RHP].targetRadian = Kubot.refAngle[8];
                joint[RKN].targetRadian = Kubot.refAngle[9]; joint[RAP].targetRadian = Kubot.refAngle[10]; joint[RAR].targetRadian = Kubot.refAngle[11];

                joint[LSP].targetRadian = Kubot.refAngle[12]; joint[LER].targetRadian = Kubot.refAngle[13]; joint[LHA].targetRadian = Kubot.refAngle[14];
                joint[RSP].targetRadian = Kubot.refAngle[15]; joint[RER].targetRadian = Kubot.refAngle[16]; joint[RHA].targetRadian = Kubot.refAngle[17];
                joint[NYA].targetRadian = Kubot.refAngle[18]; joint[HEP].targetRadian = Kubot.refAngle[19];
            }
            else if(Kubot.CommandFlag == ACT_INF_WALK || Kubot.CommandFlag == ACT_STOP_WALK || Kubot.ControlMode == CTRLMODE_WALKWALKWALK)
            {
                joint[LHY].targetRadian = joint_IK[LHY]; joint[LHR].targetRadian = joint_IK[LHR]; joint[LHP].targetRadian = joint_IK[LHP];
                joint[LKN].targetRadian = joint_IK[LKN]; joint[LAP].targetRadian = joint_IK[LAP]; joint[LAR].targetRadian = joint_IK[LAR];
                joint[RHY].targetRadian = joint_IK[RHY]; joint[RHR].targetRadian = joint_IK[RHR]; joint[RHP].targetRadian = joint_IK[RHP];
                joint[RKN].targetRadian = joint_IK[RKN]; joint[RAP].targetRadian = joint_IK[RAP]; joint[RAR].targetRadian = joint_IK[RAR];

                joint[LSP].targetRadian = Kubot.refAngle[12]; joint[LER].targetRadian = Kubot.refAngle[13]; joint[LHA].targetRadian = Kubot.refAngle[14];
                joint[RSP].targetRadian = Kubot.refAngle[15]; joint[RER].targetRadian = Kubot.refAngle[16]; joint[RHA].targetRadian = Kubot.refAngle[17];
                joint[NYA].targetRadian = Kubot.refAngle[18]; joint[HEP].targetRadian = Kubot.refAngle[19];
            }
        }
    }// 👈 1. tasktime 주기 분기문(if con_count % ...)이 완전히 끝나는 중괄호!

    // =================================================================
    // 🚀 [위치 중요] 가제보 물리 엔진이 굴러가는 '매 틱(Every single loop)'마다 
    // 관절에 PID 힘을 짱짱하게 인가하도록 여기에 함수를 배치합니다!
    // =================================================================
    jointcontroller(); 

    sensor_msgs::msg::JointState js;
    js.header.stamp = this->node_->now();

    js.name = {
        "L_Hip_yaw_joint", "L_Hip_roll_joint", "L_Hip_pitch_joint", "L_Knee_pitch_joint", "L_Ankle_pitch_joint", "L_Ankle_roll_joint",
        "R_Hip_yaw_joint", "R_Hip_roll_joint", "R_Hip_pitch_joint", "R_Knee_pitch_joint", "R_Ankle_pitch_joint", "R_Ankle_roll_joint",
        "L_Shoulder_roll_joint", "L_Elbow_pitch_joint", "L_Hand_pitch_joint",
        "R_Shoulder_roll_joint", "R_Elbow_pitch_joint", "R_Hand_pitch_joint",
        "Neck_yaw_joint", "Head_pitch_joint"
    };

    for(int i = 0; i < 20; ++i) {
        js.position.push_back(joint[i].actualRadian); 
    }

    this->P_joint_states->publish(js);
    
    con_count++; // 주기 카운트 증가 가드 유지
} // 👈 3. UpdateAlgorithm 함수가 완전히 끝나는 마지막 중괄호

void gazebo::kubot26_plugin::setjoints() 
{
    L_Hip_yaw_joint = this->model->GetJoint("L_Hip_yaw_joint");
    L_Hip_roll_joint = this->model->GetJoint("L_Hip_roll_joint");
    L_Hip_pitch_joint = this->model->GetJoint("L_Hip_pitch_joint");
    L_Knee_pitch_joint = this->model->GetJoint("L_Knee_pitch_joint");
    L_Ankle_pitch_joint = this->model->GetJoint("L_Ankle_pitch_joint");
    L_Ankle_roll_joint = this->model->GetJoint("L_Ankle_roll_joint");

    R_Hip_yaw_joint = this->model->GetJoint("R_Hip_yaw_joint");
    R_Hip_roll_joint = this->model->GetJoint("R_Hip_roll_joint");
    R_Hip_pitch_joint = this->model->GetJoint("R_Hip_pitch_joint");
    R_Knee_pitch_joint = this->model->GetJoint("R_Knee_pitch_joint");
    R_Ankle_pitch_joint = this->model->GetJoint("R_Ankle_pitch_joint");
    R_Ankle_roll_joint = this->model->GetJoint("R_Ankle_roll_joint");

    L_Shoulder_roll_joint = this->model->GetJoint("L_Shoulder_roll_joint");
    L_Elbow_pitch_joint = this->model->GetJoint("L_Elbow_pitch_joint");
    L_Hand_pitch_joint = this->model->GetJoint("L_Hand_pitch_joint");

    R_Shoulder_roll_joint = this->model->GetJoint("R_Shoulder_roll_joint");
    R_Elbow_pitch_joint = this->model->GetJoint("R_Elbow_pitch_joint");
    R_Hand_pitch_joint = this->model->GetJoint("R_Hand_pitch_joint");

    Neck_yaw_joint = this->model->GetJoint("Neck_yaw_joint");
    Head_pitch_joint = this->model->GetJoint("Head_pitch_joint");
}

void gazebo::kubot26_plugin::getjointdata()
{
    joint[LHY].actualRadian = L_Hip_yaw_joint->Position(0);
    joint[LHR].actualRadian = L_Hip_roll_joint->Position(0);
    joint[LHP].actualRadian = L_Hip_pitch_joint->Position(0);
    joint[LKN].actualRadian = L_Knee_pitch_joint->Position(0);
    joint[LAP].actualRadian = L_Ankle_pitch_joint->Position(0);
    joint[LAR].actualRadian = L_Ankle_roll_joint->Position(0);

    joint[RHY].actualRadian = R_Hip_yaw_joint->Position(0);
    joint[RHR].actualRadian = R_Hip_roll_joint->Position(0);
    joint[RHP].actualRadian = R_Hip_pitch_joint->Position(0);
    joint[RKN].actualRadian = R_Knee_pitch_joint->Position(0);
    joint[RAP].actualRadian = R_Ankle_pitch_joint->Position(0);
    joint[RAR].actualRadian = R_Ankle_roll_joint->Position(0);

    joint[LSP].actualRadian = L_Shoulder_roll_joint->Position(0);
    joint[LER].actualRadian = L_Elbow_pitch_joint->Position(0);
    joint[LHA].actualRadian = L_Hand_pitch_joint->Position(0);

    joint[RSP].actualRadian = R_Shoulder_roll_joint->Position(0);
    joint[RER].actualRadian = R_Elbow_pitch_joint->Position(0);
    joint[RHA].actualRadian = R_Hand_pitch_joint->Position(0);

    joint[NYA].actualRadian = Neck_yaw_joint->Position(0);
    joint[HEP].actualRadian = Head_pitch_joint->Position(0);
    

    for (int j = 0; j < 20; j++) { 
        joint[j].actualDegree = joint[j].actualRadian*R2D;
        test_vector(j) = joint[j].actualRadian;
    }

    joint[LHY].actualVelocity = L_Hip_yaw_joint->GetVelocity(0);
    joint[LHR].actualVelocity = L_Hip_roll_joint->GetVelocity(0);
    joint[LHP].actualVelocity = L_Hip_pitch_joint->GetVelocity(0);
    joint[LKN].actualVelocity = L_Knee_pitch_joint->GetVelocity(0);
    joint[LAP].actualVelocity = L_Ankle_pitch_joint->GetVelocity(0);
    joint[LAR].actualVelocity = L_Ankle_roll_joint->GetVelocity(0);

    joint[RHY].actualVelocity = R_Hip_yaw_joint->GetVelocity(0);
    joint[RHR].actualVelocity = R_Hip_roll_joint->GetVelocity(0);
    joint[RHP].actualVelocity = R_Hip_pitch_joint->GetVelocity(0);
    joint[RKN].actualVelocity = R_Knee_pitch_joint->GetVelocity(0);
    joint[RAP].actualVelocity = R_Ankle_pitch_joint->GetVelocity(0);
    joint[RAR].actualVelocity = R_Ankle_roll_joint->GetVelocity(0);

    joint[LSP].actualVelocity = L_Shoulder_roll_joint->GetVelocity(0);
    joint[LER].actualVelocity = L_Elbow_pitch_joint->GetVelocity(0);
    joint[LHA].actualVelocity = L_Hand_pitch_joint->GetVelocity(0);

    joint[RSP].actualVelocity = R_Shoulder_roll_joint->GetVelocity(0);
    joint[RER].actualVelocity = R_Elbow_pitch_joint->GetVelocity(0);
    joint[RHA].actualVelocity = R_Hand_pitch_joint->GetVelocity(0);

    joint[NYA].actualVelocity = Neck_yaw_joint->GetVelocity(0);
    joint[HEP].actualVelocity = Head_pitch_joint->GetVelocity(0);

    joint[LHY].actualTorque = L_Hip_yaw_joint->GetForce(0);
    joint[LHR].actualTorque = L_Hip_roll_joint->GetForce(0);
    joint[LHP].actualTorque = L_Hip_pitch_joint->GetForce(0);
    joint[LKN].actualTorque = L_Knee_pitch_joint->GetForce(0);
    joint[LAP].actualTorque = L_Ankle_pitch_joint->GetForce(0);
    joint[LAR].actualTorque = L_Ankle_roll_joint->GetForce(0);

    joint[RHY].actualTorque = R_Hip_yaw_joint->GetForce(0);
    joint[RHR].actualTorque = R_Hip_roll_joint->GetForce(0);
    joint[RHP].actualTorque = R_Hip_pitch_joint->GetForce(0);
    joint[RKN].actualTorque = R_Knee_pitch_joint->GetForce(0);
    joint[RAP].actualTorque = R_Ankle_pitch_joint->GetForce(0);
    joint[RAR].actualTorque = R_Ankle_roll_joint->GetForce(0);

    joint[LSP].actualTorque = L_Shoulder_roll_joint->GetForce(0);
    joint[LER].actualTorque = L_Elbow_pitch_joint->GetForce(0);
    joint[LHA].actualTorque = L_Hand_pitch_joint->GetForce(0);

    joint[RSP].actualTorque = R_Shoulder_roll_joint->GetForce(0);
    joint[RER].actualTorque = R_Elbow_pitch_joint->GetForce(0);
    joint[RHA].actualTorque = R_Hand_pitch_joint->GetForce(0);

    joint[NYA].actualTorque = Neck_yaw_joint->GetForce(0);
    joint[HEP].actualTorque = Head_pitch_joint->GetForce(0);

    for (int j = 0; j < nDoF; j++) {
        joint[j].actualRPM = joint[j].actualVelocity * 60. / (2 * PI);
    }  
}

void gazebo::kubot26_plugin::setsensor() 
{
    // 여러 대를 동시에 띄우면 "IMU" 라는 이름의 센서가 모델 수만큼 존재한다.
    // 전역 이름으로 찾으면 다른 로봇의 IMU 를 잡으므로 모델 스코프 이름을 쓴다.
    //   형식: <world>::<model>::<link>::<sensor>
    const std::string scoped =
        this->model->GetWorld()->Name() + "::" + this->model->GetName() + "::base_link::IMU";

    Sensor = sensors::get_sensor(scoped);
    if (!Sensor) {
        // 단독 실행 등 호환용 폴백
        Sensor = sensors::get_sensor("IMU");
    }
    IMU = std::dynamic_pointer_cast<sensors::ImuSensor>(Sensor);

    if (!IMU) {
        printf("[KUBOT] IMU 센서를 찾지 못했습니다 (%s)\n", scoped.c_str());
    }
}

void gazebo::kubot26_plugin::getsensordata()
{
    if (!IMU) return;

    double IMUdata[3];
    IMUdata[0] = IMU->Orientation().Euler()[0];
    IMUdata[1] = IMU->Orientation().Euler()[1];
    IMUdata[2] = IMU->Orientation().Euler()[2];
}

void gazebo::kubot26_plugin::jointcontroller()
{
    for (int j = 0; j < nDoF; j++) {
        joint[j].targetTorque = (joint[j].Kp * (joint[j].targetRadian - joint[j].actualRadian)) \
                              + (joint[j].Kd * (joint[j].targetVelocity - joint[j].actualVelocity));

        // 실제 모터가 낼 수 없는 토크를 시뮬레이터가 내지 않도록 스톨 토크로 포화
        const double lim = joint[j].torqueLimit;
        if (lim > 0.0) {
            if (joint[j].targetTorque >  lim) joint[j].targetTorque =  lim;
            else if (joint[j].targetTorque < -lim) joint[j].targetTorque = -lim;
        }
    }

    L_Hip_yaw_joint->SetForce(0, joint[LHY].targetTorque);
    L_Hip_roll_joint->SetForce(0, joint[LHR].targetTorque);
    L_Hip_pitch_joint->SetForce(0, joint[LHP].targetTorque);
    L_Knee_pitch_joint->SetForce(0, joint[LKN].targetTorque);
    L_Ankle_pitch_joint->SetForce(0, joint[LAP].targetTorque);        
    L_Ankle_roll_joint->SetForce(0, joint[LAR].targetTorque);         

    R_Hip_yaw_joint->SetForce(0, joint[RHY].targetTorque);
    R_Hip_roll_joint->SetForce(0, joint[RHR].targetTorque);
    R_Hip_pitch_joint->SetForce(0, joint[RHP].targetTorque);
    R_Knee_pitch_joint->SetForce(0, joint[RKN].targetTorque); 
    R_Ankle_pitch_joint->SetForce(0, joint[RAP].targetTorque);        
    R_Ankle_roll_joint->SetForce(0, joint[RAR].targetTorque); 

    L_Shoulder_roll_joint->SetForce(0, joint[LSP].targetTorque);        
    L_Elbow_pitch_joint->SetForce(0, joint[LER].targetTorque);         
    L_Hand_pitch_joint->SetForce(0, joint[LHA].targetTorque);

    R_Shoulder_roll_joint->SetForce(0, joint[RSP].targetTorque);
    R_Elbow_pitch_joint->SetForce(0, joint[RER].targetTorque);
    R_Hand_pitch_joint->SetForce(0, joint[RHA].targetTorque); 

    Neck_yaw_joint->SetForce(0, joint[NYA].targetTorque);        
    Head_pitch_joint->SetForce(0, joint[HEP].targetTorque); 
}

void gazebo::kubot26_plugin::initializejoint()
{
    joint[0].init_targetradian  = (0  *D2R);     
    joint[1].init_targetradian  = (0  *D2R);     
    joint[2].init_targetradian  = (-45*D2R);     
    joint[3].init_targetradian  = (90 *D2R);     
    joint[4].init_targetradian  = (-45*D2R);     
    joint[5].init_targetradian  = (0  *D2R);     
    joint[6].init_targetradian  = (0  *D2R);     
    joint[7].init_targetradian  = (0  *D2R);     
    joint[8].init_targetradian  = (-45*D2R);     
    joint[9].init_targetradian  = (90 *D2R);     
    joint[10].init_targetradian = (-45*D2R);     
    joint[11].init_targetradian = (0  *D2R);     

    joint[12].init_targetradian = (0  *D2R);     
    joint[13].init_targetradian = (-15*D2R);     
    joint[14].init_targetradian = (90 *D2R);     
    joint[15].init_targetradian = (0  *D2R);     
    joint[16].init_targetradian = (-15*D2R);     
    joint[17].init_targetradian = (90 *D2R);     
    joint[18].init_targetradian = (-0 *D2R);     
    joint[19].init_targetradian = (0  *D2R);     
}

void gazebo::kubot26_plugin::setjointPIDgain()
{
    joint[LHY].Kp = 130; joint[LHR].Kp = 140; joint[LHP].Kp = 160; joint[LKN].Kp = 230; joint[LAP].Kp = 130; joint[LAR].Kp = 130;
    joint[RHY].Kp = joint[LHY].Kp; joint[RHR].Kp = joint[LHR].Kp; joint[RHP].Kp = joint[LHP].Kp; joint[RKN].Kp = joint[LKN].Kp; joint[RAP].Kp = joint[LAP].Kp; joint[RAR].Kp = joint[LAR].Kp;

    joint[LHY].Kd = 0.06; joint[LHR].Kd = 0.08; joint[LHP].Kd = 0.11; joint[LKN].Kd = 0.10; joint[LAP].Kd = 0.06; joint[LAR].Kd = 0.06;
    joint[RHY].Kd = joint[LHY].Kd; joint[RHR].Kd = joint[LHR].Kd; joint[RHP].Kd = joint[LHP].Kd; joint[RKN].Kd = joint[LKN].Kd; joint[RAP].Kd = joint[LAP].Kd; joint[RAR].Kd = joint[LAR].Kd;

    // 팔/목 게인. 기존 값은 전부 Kp=10 자리표시값이라 중력에 처졌다.
    // 관절별 원위 체인 관성 I 와 1ms 스텝 안정조건(w*dt = sqrt(Kp/I)*dt << 2)을
    // 보고 잡았다. 실제 출력은 jointcontroller() 에서 모터 스톨 토크로 포화된다.
    //   Shoulder_roll I=1.13e-2 -> w=103 rad/s (w*dt=0.10)
    //   Elbow_pitch   I=7.28e-4 -> w=262      (0.26)
    //   Hand_pitch    I=1.04e-3 -> w=170      (0.17)
    //   Neck_yaw      I=4.45e-4 -> w=212      (0.21)
    //   Head_pitch    I=1.58e-3 -> w=159      (0.16)
    joint[LSP].Kp = 120;  joint[RSP].Kp = 120;   // Shoulder_roll  (MX-64)
    joint[LER].Kp = 50;   joint[RER].Kp = 50;    // Elbow_pitch    (MX-64)
    joint[LHA].Kp = 30;   joint[RHA].Kp = 30;    // Hand_pitch     (MX-28)
    joint[NYA].Kp = 20;                          // Neck_yaw       (MX-28)
    joint[HEP].Kp = 40;                          // Head_pitch     (MX-28)

    joint[LSP].Kd = 0.10; joint[RSP].Kd = 0.10;
    joint[LER].Kd = 0.04; joint[RER].Kd = 0.04;
    joint[LHA].Kd = 0.025; joint[RHA].Kd = 0.025;
    joint[NYA].Kd = 0.02;
    joint[HEP].Kd = 0.03;

    // ===== 실제 장착 모터의 스톨 토크 (Dynamixel MX 시리즈) =====
    // MX-106 : 8.4 Nm  - Hip_pitch, Hip_roll, Knee_pitch, Ankle_pitch, Ankle_roll
    // MX-64  : 6.0 Nm  - Hip_yaw, Shoulder_roll, Elbow_pitch
    // MX-28  : 2.5 Nm  - Hand_pitch, Neck_yaw, Head_pitch
    const double MX106 = 8.4, MX64 = 6.0, MX28 = 2.5;

    joint[LHP].torqueLimit = MX106; joint[LHR].torqueLimit = MX106;
    joint[LKN].torqueLimit = MX106; joint[LAP].torqueLimit = MX106; joint[LAR].torqueLimit = MX106;
    joint[RHP].torqueLimit = MX106; joint[RHR].torqueLimit = MX106;
    joint[RKN].torqueLimit = MX106; joint[RAP].torqueLimit = MX106; joint[RAR].torqueLimit = MX106;

    joint[LHY].torqueLimit = MX64;  joint[RHY].torqueLimit = MX64;
    joint[LSP].torqueLimit = MX64;  joint[LER].torqueLimit = MX64;
    joint[RSP].torqueLimit = MX64;  joint[RER].torqueLimit = MX64;

    joint[LHA].torqueLimit = MX28;  joint[RHA].torqueLimit = MX28;
    joint[NYA].torqueLimit = MX28;  joint[HEP].torqueLimit = MX28;
}

// 생략되었던 Joy 및 Callback 함수들 본형 유지 매핑


// 1. 모드 번호 수신 함수 (RQT에서 1, 2, 4 등 쏘면 동작)
// 1. 모드 제어 콜백 함수 (ROS1 원본 100% 이식)
void gazebo::kubot26_plugin::KubotMode(const std_msgs::msg::Int32::SharedPtr msg)
{
    ControlMode_by_ROS = msg->data;
    printf("[KUBOT TOPIC] 모드 번호 %d 수신됨!\n", ControlMode_by_ROS);

    switch (ControlMode_by_ROS) {

    case 1:
        Kubot.startTime = Kubot.realTime;
        Kubot.ControlMode = CTRLMODE_HOMEPOSE;
        Kubot.CommandFlag = RETURN_HOMEPOSE;
        printf("Kubot home pose! \n");
        Kubot.Move_current = false;
        ControlMode_by_ROS = 0;
        break;

    case 2:
        Kubot.startTime = Kubot.realTime;
        Kubot.ControlMode = CTRLMODE_WALKREADY;
        Kubot.CommandFlag = GOTO_WALK_READY_POS;
        printf("WALK READY START! \n");
        Kubot.Move_current = false;
        ControlMode_by_ROS = 0;
        break;

    case 3:
        Kubot.startTime = Kubot.realTime;
        Kubot.ControlMode = CTRLMODE_RAISELEFTARM;
        Kubot.CommandFlag = Raise_Left_Arm;
        printf("Raise_Left_Arm!\n");
        Kubot.Move_current = false;
        ControlMode_by_ROS = 0;
        break;

    case 4:
        Kubot.startTime = Kubot.realTime;
        Kubot.ControlMode = CTRLMODE_WALKWALKWALK;
        Kubot.CommandFlag = WALKTHREETIMES;
        printf("WALKINGTHREETIMES!\n");
        Kubot.Move_current = false;
        ControlMode_by_ROS = 0;
        break;

    case 5:
        Kubot.startTime = Kubot.realTime;
        Kubot.ControlMode = CTRLMODE_KICKREADYPOSE;
        Kubot.CommandFlag = kickreadypose;
        printf("kickreadypose!\n");
        Kubot.Move_current = false;
        ControlMode_by_ROS = 0;
        break;

    case 7:
        Kubot.startTime = Kubot.realTime;
        Kubot.ControlMode = CTRLMODE_PHYSICSMODE;
        Kubot.CommandFlag = walkingphysics;
        printf("walkingphysics!\n");
        Kubot.Move_current = false;
        ControlMode_by_ROS = 0;
        break;

    case 25:
        Kubot.motion_1 = false;
        Kubot.startTime = Kubot.realTime;
        Kubot.ControlMode = CTRLMODE_STANDUP_TEST;
        Kubot.CommandFlag = ACT_STANDUP_FRONT;
        printf("STANDUP FRONT MOVE START! \n");
        Kubot.Move_current = false;
        ControlMode_by_ROS = 0;
        Kubot.STANDUP_READY = true;
        Kubot.STANDUP_ING = false;
        break;

    case 26:
        Kubot.motion_1 = false;
        Kubot.startTime = Kubot.realTime;
        Kubot.ControlMode = CTRLMODE_STANDUP_TEST;
        Kubot.CommandFlag = ACT_STANDUP_BACK;
        printf("STANDUP BACK MOVE START! \n");
        Kubot.Move_current = false;
        ControlMode_by_ROS = 0;
        Kubot.STANDUP_READY = true;
        Kubot.STANDUP_ING = false;
        break;

    case 44:
        Kubot.motion_1 = false; // 연속 킥 실행을 위한 1단계 리셋 구문 보완
        Kubot.startTime = Kubot.realTime;
        Kubot.ControlMode = CTRLMODE_MOTION;
        Kubot.CommandFlag = ACT_KICK_L;
        printf("KICKTHEBALL (LEFT)!\n");
        Kubot.Move_current = false;
        ControlMode_by_ROS = 0;
        Kubot.MOTION_READY = true;
        Kubot.MOTION_ING = false;
        break;

    case 45:
        Kubot.motion_1 = false; // 연속 킥 실행을 위한 1단계 리셋 구문 보완
        Kubot.startTime = Kubot.realTime;
        Kubot.ControlMode = CTRLMODE_MOTION;
        Kubot.CommandFlag = ACT_KICK_R;
        printf("KICK MOTION R START! \n");
        Kubot.Move_current = false;
        ControlMode_by_ROS = 0;
        Kubot.MOTION_READY = true;
        Kubot.MOTION_ING = false;
        break;

    case 70:
        Kubot.startTime = Kubot.realTime;
        Kubot.ControlMode = CTRLMODE_TURNING_L;
        Kubot.CommandFlag = ACT_TURNING_L;
        printf("TURN_L START! \n");
        Kubot.Move_current = false;
        ControlMode_by_ROS = 0;
        Kubot.MOTION_READY = true;
        Kubot.MOTION_ING = false;
        break;

    case 71:
        Kubot.startTime = Kubot.realTime;
        Kubot.ControlMode = CTRLMODE_TURNING_R;
        Kubot.CommandFlag = ACT_TURNING_R;
        printf("TURN_R START! \n");
        Kubot.Move_current = false;
        ControlMode_by_ROS = 0;
        Kubot.MOTION_READY = true;
        Kubot.MOTION_ING = false;
        break;

    case 100:
        Kubot.startTime = Kubot.realTime;
        Kubot.ControlMode = CTRLMODE_GOALKEEPERMODE;
        Kubot.CommandFlag = GOALKEEPER;
        printf("GOALKEEPER!\n");
        Kubot.Move_current = false;
        ControlMode_by_ROS = 0;
        break;

    case 99: // save file
        printf("save file START!\n");
        Kubot.save_file(50000);
        printf("save file DONE!\n");
        ControlMode_by_ROS = 0;
        break;

    default:
        break;
    }
}

// 2. 파라미터 제어 메시지 콜백 함수 (ROS2 Humble 규격)
void gazebo::kubot26_plugin::Kubot_control_callback(const kubot26_pkgs::msg::KubotControlMsgs::SharedPtr msg)
{
    Kubot.steps_msgs      = msg->steps;
    Kubot.FBSize_msgs     = msg->fb_size;
    Kubot.LRSize_msgs     = msg->lr_size;
    Kubot.TurnSize_msgs   = msg->turn_size;
    Kubot.footHeight_msgs = msg->foot_height;
    Kubot.stop_msgs       = msg->stop;

    printf("======Subscriber_read_data=====\n");
    printf("steps : %d, FBSize : %lf, LRSize : %lf, TurnSize : %lf, footHeight : %lf, stop : %d\n", 
           Kubot.steps_msgs, Kubot.FBSize_msgs, Kubot.LRSize_msgs, Kubot.TurnSize_msgs, Kubot.footHeight_msgs, Kubot.stop_msgs);
}

// 3. 조이스틱 신호 수신 함수
void gazebo::kubot26_plugin::Kubot_joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    // 조이스틱 연동 로직
    printf("[KUBOT TOPIC] 조이스틱 신호 수신\n");
}