
#ifndef _READWRITE_EXAMPLE_H_
#define _READWRITE_EXAMPLE_H_

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/ColorRGBA.h>
#include <sensor_msgs/JointState.h>

#include <robotis_controller/ControlWrite.h>
#include <robotis_controller/ControlTorque.h>
#include <robotis_controller/PublishPosition.h>

namespace robotis_example
{

class ReadWrite
{
public:
    enum {
        LEFT_TO_RIGHT,
        RIGHT_TO_LEFT
    };

    int mode;
    bool manager_ready;
    ros::NodeHandle nh_;

    std::string joint_name[20];

    ReadWrite(ros::NodeHandle nh);
    ~ReadWrite();
    ros::Publisher joint_states_pub;
    ros::Publisher control_write_pub;
    ros::Subscriber manager_ready_sub;
    ros::Subscriber eye_led_color_sub;
    ros::Subscriber head_led_color_sub;
    ros::Publisher pp_pub;
    ros::Publisher ct_pub;
    ros::Subscriber op2_joint_states_sub;

    void control_write(int id, int addr, int length, int value);
    void manager_ready_callback(const std_msgs::Bool::ConstPtr& msg);
    void eye_led_color_callback(const std_msgs::ColorRGBA::ConstPtr& msg);
    void head_led_color_callback(const std_msgs::ColorRGBA::ConstPtr& msg);
    void op2_joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg);
};

}   // namespace robotis_example

#endif  //_READWRITE_EXAMPLE_H_
