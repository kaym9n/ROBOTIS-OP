#ifndef _ACTION_SCRIPT_H_
#define _ACTION_SCRIPT_H_

#include <stdio.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>

#include <boost/thread.hpp>

#include <robotis_controller/ControlWrite.h>
#include <robotis_controller/ControlTorque.h>
#include <robotis_controller/PublishPosition.h>

namespace ROBOTIS
{

class ActionScript
{
public:
    enum
    {
        ID_R_SHOULDER_PITCH     = 1,
        ID_L_SHOULDER_PITCH     = 2,
        ID_R_SHOULDER_ROLL      = 3,
        ID_L_SHOULDER_ROLL      = 4,
        ID_R_ELBOW              = 5,
        ID_L_ELBOW              = 6,
        ID_R_HIP_YAW            = 7,
        ID_L_HIP_YAW            = 8,
        ID_R_HIP_ROLL           = 9,
        ID_L_HIP_ROLL           = 10,
        ID_R_HIP_PITCH          = 11,
        ID_L_HIP_PITCH          = 12,
        ID_R_KNEE               = 13,
        ID_L_KNEE               = 14,
        ID_R_ANKLE_PITCH        = 15,
        ID_L_ANKLE_PITCH        = 16,
        ID_R_ANKLE_ROLL         = 17,
        ID_L_ANKLE_ROLL         = 18,
        ID_HEAD_PAN             = 19,
        ID_HEAD_TILT            = 20,
        NUMBER_OF_JOINTS
    };
    enum
    {
        MAXNUM_PAGE = 256,
        MAXNUM_STEP = 7,
        MAXNUM_NAME = 13
    };

    enum
    {
        SPEED_BASE_SCHEDULE = 0,
        TIME_BASE_SCHEDULE = 0x0a
    };

    enum
    {
        INVALID_BIT_MASK	= 0x4000,
        TORQUE_OFF_BIT_MASK	= 0x2000
    };

    enum
    {
        NotCount    = -1,
        NoPort      = 0,
    };

    typedef struct // Header Structure (total 64unsigned char)
    {
        unsigned char name[MAXNUM_NAME+1]; // Name             0~13
        unsigned char reserved1;        // Reserved1        14
        unsigned char repeat;           // Repeat count     15
        unsigned char schedule;         // schedule         16
        unsigned char reserved2[3];     // reserved2        17~19
        unsigned char stepnum;          // Number of step   20
        unsigned char reserved3;        // reserved3        21
        unsigned char speed;            // Speed            22
        unsigned char reserved4;        // reserved4        23
        unsigned char accel;            // Acceleration time 24
        unsigned char next;             // Link to next     25
        unsigned char exit;             // Link to exit     26
        unsigned char reserved5[4];     // reserved5        27~30
        unsigned char checksum;         // checksum         31
        unsigned char slope[31];        // CW/CCW compliance slope  32~62
        unsigned char reserved6;        // reserved6        63
    } PAGEHEADER;

    typedef struct // Step Structure (total 64unsigned char)
    {
        unsigned short position[31];    // Joint position   0~61
        unsigned char pause;            // Pause time       62
        unsigned char time;             // Time             63
    } STEP;

    typedef struct // Page Structure (total 512unsigned char)
    {
        PAGEHEADER header;          // Page header  0~64
        STEP step[MAXNUM_STEP];		// Page step    65~511
    } PAGE;

    const int CENTER_VALUE;
    const int MAX_VALUE;
    const double MIN_ANGLE;
    const double MAX_ANGLE;
    const double RATIO_VALUE2ANGLE;
    const double RATIO_ANGLE2VALUE;

    /*
    const int CENTER_VALUE = 2048;
    const int MAX_VALUE = 4095;
    const double MIN_ANGLE = -180.0; // degree
    const double MAX_ANGLE = 180.0; // degree
    const double RATIO_VALUE2ANGLE = 0.088; // 360 / 4096
    const double RATIO_ANGLE2VALUE = 11.378; // 4096 / 360
*/
public:
    bool DEBUG_PRINT;

    ActionScript(ros::NodeHandle nh_, ros::NodeHandle param_nh_);
    ~ActionScript();

    // static Action* GetInstance() { return m_UniqueInstance; }

    void Initialize();
    void Process();
    bool LoadFile(char* filename);
    bool CreateFile(char* filename);
    bool Start(int iPage);
    bool Start(char* namePage);
    bool Start(int index, PAGE *pPage);
    void Stop();
    void Brake();
    bool IsRunning();
    bool IsRunning(int *iPage, int *iStep);
    bool LoadPage(int index, PAGE *pPage);
    bool SavePage(int index, PAGE *pPage);
    void ResetPage(PAGE *pPage);

private:

    /////////////// Enum

    /**************************************
    * Section             /----\
    *                    /|    |\
    *        /+---------/ |    | \
    *       / |        |  |    |  \
    * -----/  |        |  |    |   \----
    *      PRE  MAIN   PRE MAIN POST PAUSE
    ***************************************/
    enum{ PRE_SECTION, MAIN_SECTION, POST_SECTION, PAUSE_SECTION };
    enum{ ZERO_FINISH, NONE_ZERO_FINISH};

    ros::NodeHandle nh;
    ros::NodeHandle param_nh;
    ros::Subscriber action_control_sub;
    ros::Subscriber current_joint_sub;
    ros::Publisher  controller_joint_pub;
    std::string joint_names[NUMBER_OF_JOINTS];
    ros::Publisher control_write_pub;
    ros::Subscriber manager_ready_sub;
    ros::Publisher pp_pub;
    ros::Publisher ct_pub;

    sensor_msgs::JointState current_joint_state;
    sensor_msgs::JointState desired_position;

    FILE* m_ActionFile;
    PAGE m_PlayPage;
    PAGE m_NextPlayPage;
    STEP m_CurrentStep;

    bool m_startNode;
    bool manager_ready;
    int m_portNum;
    int m_IndexPlayingPage;
    bool m_FirstDrivingStart;
    int m_PageStepCount;
    bool m_Playing;
    bool m_StopPlaying;
    bool m_PlayingFinished;
    boost::thread comm_thread, ros_thread;

    // Action();

    bool initManager();
    bool VerifyChecksum( PAGE *pPage );
    void SetChecksum( PAGE *pPage );
    bool getJointName(const int& id, std::string &joint_name);
    bool isJointNameInParam();
    bool getJointPosition();
    void motionControlCallback(const std_msgs::Int16::ConstPtr &msg);
    void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
    void manager_ready_callback(const std_msgs::Bool::ConstPtr& msg);
    void comm_thread_proc();
    void ros_thread_proc();

    void control_write(int id, int addr, int length, int value);

    int Angle2Value(double angle) { return (int)(angle*RATIO_ANGLE2VALUE)+CENTER_VALUE; }
    double Value2Angle(int value) { return (double)(value-CENTER_VALUE)*RATIO_VALUE2ANGLE; }
    double Rad2Deg(double rad)  { return rad * 180.0 / M_PI; }
    double Deg2Rad(double deg)  { return deg * M_PI / 180.0; }

};

}   // namespace robotis_example

#endif  //_ACTION_SCRIPT_H_
