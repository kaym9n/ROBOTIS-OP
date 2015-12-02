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

enum {
    LEFT_TO_RIGHT,
    RIGHT_TO_LEFT
};

int mode = RIGHT_TO_LEFT;
bool manager_ready = false;

std::string joint_name[20] = {
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
        "head_tilt"
};

ros::Publisher joint_states_pub;
ros::Publisher control_write_pub;

void control_write(int id, int addr, int length, int value)
{
    robotis_controller::ControlWrite cw;
    cw.id = id;
    cw.addr = addr;
    cw.length = length;
    cw.value = value;
    control_write_pub.publish(cw);
}

void manager_ready_callback(const std_msgs::Bool::ConstPtr& msg)
{
    manager_ready = true;
}

void eye_led_color_callback(const std_msgs::ColorRGBA::ConstPtr& msg)
{
    int r = 0, g = 0, b = 0, rgb = 0;
    r = ((int)msg->r & 0xFF) >> 3;
    g = ((int)msg->g & 0xFF) >> 3;
    b = ((int)msg->b & 0xFF) >> 3;
    rgb = r + (g << 5) + (b << 10);
    control_write(200, 28, 2, rgb);
}

void head_led_color_callback(const std_msgs::ColorRGBA::ConstPtr& msg)
{
    int r = 0, g = 0, b = 0, rgb = 0;
    r = ((int)msg->r & 0xFF) >> 3;
    g = ((int)msg->g & 0xFF) >> 3;
    b = ((int)msg->b & 0xFF) >> 3;
    rgb = r + (g << 5) + (b << 10);
    control_write(200, 26, 2, rgb);
}

void op2_joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    sensor_msgs::JointState joint_states;

    if(mode == RIGHT_TO_LEFT)
    {
        for(unsigned int idx = 0; idx < msg->name.size(); idx++)
        {
            if(msg->name[idx] == "r_sho_pitch")
            {
                joint_states.name.push_back("l_sho_pitch");
                joint_states.position.push_back(-msg->position[idx]);
            }
            else if(msg->name[idx] == "r_sho_roll")
            {
                joint_states.name.push_back("l_sho_roll");
                joint_states.position.push_back(-msg->position[idx]);
            }
            else if(msg->name[idx] == "r_el")
            {
                joint_states.name.push_back("l_el");
                joint_states.position.push_back(-msg->position[idx]);
            }
        }
    }
    else if(mode == LEFT_TO_RIGHT)
    {
        for(unsigned int idx = 0; idx < msg->name.size(); idx++)
        {
            if(msg->name[idx] == "l_sho_pitch")
            {
                joint_states.name.push_back("r_sho_pitch");
                joint_states.position.push_back(-msg->position[idx]);
            }
            else if(msg->name[idx] == "l_sho_roll")
            {
                joint_states.name.push_back("r_sho_roll");
                joint_states.position.push_back(-msg->position[idx]);
            }
            else if(msg->name[idx] == "l_el")
            {
                joint_states.name.push_back("r_el");
                joint_states.position.push_back(-msg->position[idx]);
            }
        }
    }

    joint_states_pub.publish(joint_states);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "example_read_write");

    ros::NodeHandle nh;

    ros::Subscriber manager_ready_sub   = nh.subscribe("/manager_ready", 10, manager_ready_callback);
    ros::Subscriber eye_led_color_sub   = nh.subscribe("/eye_led_color", 10, eye_led_color_callback);
    ros::Subscriber head_led_color_sub  = nh.subscribe("/head_led_color", 10, head_led_color_callback);

    ros::Publisher pp_pub = nh.advertise<robotis_controller::PublishPosition>("/publish_position", 10);
    ros::Publisher ct_pub = nh.advertise<robotis_controller::ControlTorque>("/control_torque", 10);

    joint_states_pub = nh.advertise<sensor_msgs::JointState>("/controller_joint_states", 10);
    control_write_pub = nh.advertise<robotis_controller::ControlWrite>("/control_write", 10);

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
    sensor_msgs::JointState init_joint_states;
    for(int i = 0; i < 20; i++)
        init_joint_states.name.push_back(joint_name[i]);
    init_joint_states.position.push_back(-0.840621);
    init_joint_states.position.push_back( 0.719437);
    init_joint_states.position.push_back(-0.328272);
    init_joint_states.position.push_back( 0.360485);
    init_joint_states.position.push_back( 0.509282);
    init_joint_states.position.push_back(-0.518486);
    init_joint_states.position.push_back(-0.007670);
    init_joint_states.position.push_back(-0.023010);
    init_joint_states.position.push_back( 0.013806);
    init_joint_states.position.push_back(-0.007670);
    init_joint_states.position.push_back(-1.182699);
    init_joint_states.position.push_back( 1.148952);
    init_joint_states.position.push_back( 2.247282);
    init_joint_states.position.push_back(-2.265690);
    init_joint_states.position.push_back( 1.219515);
    init_joint_states.position.push_back(-1.239456);
    init_joint_states.position.push_back( 0.044485);
    init_joint_states.position.push_back(-0.016874);
    init_joint_states.position.push_back( 0.003068);
    init_joint_states.position.push_back( 0.191748);
    joint_states_pub.publish(init_joint_states);
    sleep(3);

    // set the goal velocity of all dynamixels to 0
    control_write(254, 32, 2, 0);
    /***** init pose end *****/


    // Right Arm Torque OFF
    robotis_controller::ControlTorque ct;
    ct.name.push_back("r_sho_pitch");
    ct.enable.push_back(false);
    ct.name.push_back("r_sho_roll");
    ct.enable.push_back(false);
    ct.name.push_back("r_el");
    ct.enable.push_back(false);
    ct.name.push_back("head_pan");
    ct.enable.push_back(false);
    ct.name.push_back("head_tilt");
    ct.enable.push_back(false);
    ct_pub.publish(ct);


    // publish all joint position
    robotis_controller::PublishPosition pp;
    for(int i = 0; i < 20; i++)
    {
        pp.name.push_back(joint_name[i]);
        pp.publish.push_back(true);
    }
    pp_pub.publish(pp);


    ros::Subscriber op2_joint_states_sub    = nh.subscribe("/robot_joint_states", 1, op2_joint_states_callback);

    ros::spin();
    return 0;
}


