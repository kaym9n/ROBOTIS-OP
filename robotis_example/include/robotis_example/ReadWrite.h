/*
 *=====================================================
 * File   :  read_write.cpp
 * Author :  zerom <zerom@robotis.com>
 * Copyright (C) ROBOTIS, 2015
 *=====================================================
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

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


