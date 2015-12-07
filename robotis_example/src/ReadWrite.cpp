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

#include "robotis_example/ReadWrite.h"

namespace robotis_example
{

ReadWrite::ReadWrite(ros::NodeHandle nh)
	:nh_(nh), mode(RIGHT_TO_LEFT), manager_ready(false), joint_name({
        "r_sho_pitch",
        "l_sho_pitch",
        "r_sho_roll",
        "l_sho_roll",
        "r_el","l_el",
        "r_hip_yaw",
        "l_hip_yaw",
        "r_hip_roll",
        "l_hip_roll",
        "r_hip_pitch",
        "l_hip_pitch",
        "r_knee",
        "l_knee",
        "r_ank_pitch",
        "l_ank_pitch",
        "r_ank_roll",
        "l_ank_roll",
        "head_pan",
        "head_tilt" })
{
    manager_ready_sub   = nh_.subscribe("/manager_ready", 10, &ReadWrite::manager_ready_callback, this);
    eye_led_color_sub   = nh_.subscribe("/eye_led_color", 10, &ReadWrite::eye_led_color_callback, this);
    head_led_color_sub  = nh_.subscribe("/head_led_color", 10, &ReadWrite::head_led_color_callback, this);

    pp_pub = nh_.advertise<robotis_controller::PublishPosition>("/publish_position", 10);
    ct_pub = nh_.advertise<robotis_controller::ControlTorque>("/control_torque", 10);

    joint_states_pub = nh_.advertise<sensor_msgs::JointState>("/controller_joint_states", 10);
    control_write_pub = nh_.advertise<robotis_controller::ControlWrite>("/control_write", 10);

    // Wait for robotis_manager initialization
    while(manager_ready == false)
        ros::spinOnce();
    sleep(2);

    // CM-740 control table address 24 : Dynamixel Power
    control_write(200, 24, 1, 1);
    usleep(100*1000);

    // Enable the torque of all dynamixels.
    control_write(254, 24, 1, 1);
    usleep(100*1000);

    /***** init pose start *****/
    // set the goal velocity of all dynamixels to 100
    control_write(254, 32, 2, 100);

    // init pose (sit down)
    sensor_msgs::JointState _init_joint_states;
    for(int i = 0; i < 20; i++)
        _init_joint_states.name.push_back(joint_name[i]);
    _init_joint_states.position.push_back(-0.840621);
    _init_joint_states.position.push_back( 0.719437);
    _init_joint_states.position.push_back(-0.328272);
    _init_joint_states.position.push_back( 0.360485);
    _init_joint_states.position.push_back( 0.509282);
    _init_joint_states.position.push_back(-0.518486);
    _init_joint_states.position.push_back(-0.007670);
    _init_joint_states.position.push_back(-0.023010);
    _init_joint_states.position.push_back( 0.013806);
    _init_joint_states.position.push_back(-0.007670);
    _init_joint_states.position.push_back(-1.182699);
    _init_joint_states.position.push_back( 1.148952);
    _init_joint_states.position.push_back( 2.247282);
    _init_joint_states.position.push_back(-2.265690);
    _init_joint_states.position.push_back( 1.219515);
    _init_joint_states.position.push_back(-1.239456);
    _init_joint_states.position.push_back( 0.044485);
    _init_joint_states.position.push_back(-0.016874);
    _init_joint_states.position.push_back( 0.003068);
    _init_joint_states.position.push_back( 0.191748);
    joint_states_pub.publish(_init_joint_states);
    sleep(3);

    // set the goal velocity of all dynamixels to 0
    control_write(254, 32, 2, 0);
    /***** init pose end *****/


    // Right Arm Torque OFF
    robotis_controller::ControlTorque _ct;
    _ct.name.push_back("r_sho_pitch");
    _ct.enable.push_back(false);
    _ct.name.push_back("r_sho_roll");
    _ct.enable.push_back(false);
    _ct.name.push_back("r_el");
    _ct.enable.push_back(false);
    _ct.name.push_back("head_pan");
    _ct.enable.push_back(false);
    _ct.name.push_back("head_tilt");
    _ct.enable.push_back(false);
    ct_pub.publish(_ct);


    // publish all joint position
    robotis_controller::PublishPosition _pp;
    for(int i = 0; i < 20; i++)
    {
        _pp.name.push_back(joint_name[i]);
        _pp.publish.push_back(true);
    }
    pp_pub.publish(_pp);

   op2_joint_states_sub    = nh.subscribe("/robot_joint_states", 1, &ReadWrite::op2_joint_states_callback, this);

}

ReadWrite::~ReadWrite()
{

}

void ReadWrite::control_write(int id, int addr, int length, int value)
{
    robotis_controller::ControlWrite _cw;
    _cw.id = id;
    _cw.addr = addr;
    _cw.length = length;
    _cw.value = value;
    control_write_pub.publish(_cw);
}

void ReadWrite::manager_ready_callback(const std_msgs::Bool::ConstPtr& msg)
{
    manager_ready = true;
}

void ReadWrite::eye_led_color_callback(const std_msgs::ColorRGBA::ConstPtr& msg)
{
    int _r = 0, _g = 0, _b = 0, _rgb = 0;
    _r = ((int)msg->r & 0xFF) >> 3;
    _g = ((int)msg->g & 0xFF) >> 3;
    _b = ((int)msg->b & 0xFF) >> 3;
    _rgb = _r + (_g << 5) + (_b << 10);
    control_write(200, 28, 2, _rgb);
}

void ReadWrite::head_led_color_callback(const std_msgs::ColorRGBA::ConstPtr& msg)
{
    int _r = 0, _g = 0, _b = 0, _rgb = 0;
    _r = ((int)msg->r & 0xFF) >> 3;
    _g = ((int)msg->g & 0xFF) >> 3;
    _b = ((int)msg->b & 0xFF) >> 3;
    _rgb = _r + (_g << 5) + (_b << 10);
    control_write(200, 26, 2, _rgb);
}

void ReadWrite::op2_joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    // ros::Time _now = ros::Time::now();
    // ros::Duration _dur = _now - msg->header.stamp;
    // ROS_INFO_STREAM("e_received | " << _dur);

    sensor_msgs::JointState _joint_states;

    if(mode == RIGHT_TO_LEFT)
    {
        for(unsigned int idx = 0; idx < msg->name.size(); idx++)
        {
            if(msg->name[idx] == "r_sho_pitch")
            {
                _joint_states.name.push_back("l_sho_pitch");
                _joint_states.position.push_back(-msg->position[idx]);
            }
            else if(msg->name[idx] == "r_sho_roll")
            {
                _joint_states.name.push_back("l_sho_roll");
                _joint_states.position.push_back(-msg->position[idx]);
            }
            else if(msg->name[idx] == "r_el")
            {
                _joint_states.name.push_back("l_el");
                _joint_states.position.push_back(-msg->position[idx]);
            }
        }
    }
    else if(mode == LEFT_TO_RIGHT)
    {
        for(unsigned int idx = 0; idx < msg->name.size(); idx++)
        {
            if(msg->name[idx] == "l_sho_pitch")
            {
                _joint_states.name.push_back("r_sho_pitch");
                _joint_states.position.push_back(-msg->position[idx]);
            }
            else if(msg->name[idx] == "l_sho_roll")
            {
                _joint_states.name.push_back("r_sho_roll");
                _joint_states.position.push_back(-msg->position[idx]);
            }
            else if(msg->name[idx] == "l_el")
            {
                _joint_states.name.push_back("r_el");
                _joint_states.position.push_back(-msg->position[idx]);
            }
        }
    }

    // ROS_INFO("send");
    _joint_states.header.stamp = ros::Time::now();
    joint_states_pub.publish(_joint_states);
}

}       // namespace robotis_example

