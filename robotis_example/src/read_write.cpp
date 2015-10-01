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

#include <unistd.h>
#include <sensor_msgs/JointState.h>

#include "DynamixelController.h"
#include "packet_control/GroupHandler.h"

using namespace ROBOTIS;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "example_read_write");

    ros::NodeHandle nh("~");
    ros::Publisher joint_states_pub = nh.advertise<sensor_msgs::JointState>("/op2_joint_states", 1);

    DynamixelController *dxl_controller = new DynamixelController();
    // check .launch file parameter
    dxl_controller->initialize();

    int result = COMM_SUCCESS;

    GroupHandler grp_handler(dxl_controller);
    for(int c = 0; c < dxl_controller->idList.size(); c++)
    {
        int _id = dxl_controller->idList[c];
        grp_handler.pushBulkRead(_id, dxl_controller->getDynamixel(_id)->ADDR_PRESENT_POSITION);
    }

    // CM-740 control table address 24 : Dynamixel Power
    result = dxl_controller->write(200, 24, 1, LENGTH_1BYTE);
    if(result != COMM_SUCCESS)
        ROS_INFO("Dynamixel Power ON failed! : %d ", result);

    usleep(200*1000);

    // Enable the torque of all dynamixels.
    result = dxl_controller->write(BROADCAST_ID, 24, 1, LENGTH_1BYTE);
    if(result != COMM_SUCCESS)
        ROS_INFO("Failed to enable the torque of all dynamixels : %d ", result);

    usleep(200*1000);

    /***** init pose start *****/
    // set the goal velocity of all dynamixels to 100
    dxl_controller->write(BROADCAST_ID, dxl_controller->getDynamixel(1)->ADDR_GOAL_VELOCITY, 100, LENGTH_2BYTE);

    // set init pose position (sit down pose)
    dxl_controller->setGoalPositionRadian(1,  -0.840621);
    dxl_controller->setGoalPositionRadian(2,   0.719437);
    dxl_controller->setGoalPositionRadian(3,  -0.328272);
    dxl_controller->setGoalPositionRadian(4,   0.360485);
    dxl_controller->setGoalPositionRadian(5,   0.509282);
    dxl_controller->setGoalPositionRadian(6,  -0.518486);
    dxl_controller->setGoalPositionRadian(7,  -0.007670);
    dxl_controller->setGoalPositionRadian(8,  -0.023010);
    dxl_controller->setGoalPositionRadian(9,   0.013806);
    dxl_controller->setGoalPositionRadian(10, -0.007670);
    dxl_controller->setGoalPositionRadian(11, -1.182699);
    dxl_controller->setGoalPositionRadian(12,  1.148952);
    dxl_controller->setGoalPositionRadian(13,  2.247282);
    dxl_controller->setGoalPositionRadian(14, -2.265690);
    dxl_controller->setGoalPositionRadian(15,  1.219515);
    dxl_controller->setGoalPositionRadian(16, -1.239456);
    dxl_controller->setGoalPositionRadian(17,  0.044485);
    dxl_controller->setGoalPositionRadian(18, -0.016874);
    dxl_controller->setGoalPositionRadian(19,  0.003068);
    dxl_controller->setGoalPositionRadian(20,  0.191748);

    sleep(4);
    // set the goal velocity of all dynamixels to 0
    dxl_controller->write(BROADCAST_ID, 32, 0, LENGTH_2BYTE);
    /***** init pose end *****/

    // disable the torque of right arm joints.
    dxl_controller->setTorqueEnable(1, 0);
    dxl_controller->setTorqueEnable(3, 0);
    dxl_controller->setTorqueEnable(5, 0);

    sensor_msgs::JointState joint_states;
    joint_states.name.resize(dxl_controller->idList.size());
    joint_states.position.resize(dxl_controller->idList.size());

    while(ros::ok())
    {
        // Run BulkRead
        grp_handler.runBulkRead();

        // publish position of all joints
        for(int x = 0; x < dxl_controller->idList.size(); x++)
        {
            long    _pos    = 0;
            int     _id     = dxl_controller->idList[x];
            if(grp_handler.getReadData(_id, dxl_controller->getDynamixel(_id)->ADDR_PRESENT_POSITION, &_pos) == true)
            {
                joint_states.name[x]        = dxl_controller->getDynamixel(_id)->getJointName();
                joint_states.position[x]    = dxl_controller->getDynamixel(_id)->value2Rad(_pos);
            }
        }
        joint_states.header.stamp = ros::Time::now();
        joint_states_pub.publish(joint_states);

        // read position from right arm & write mirroring position to left arm
        double pp;
        dxl_controller->getPresentPositionRadian(1, &pp);
        dxl_controller->setGoalPositionRadian(2, -pp);

        dxl_controller->getPresentPositionRadian(3, &pp);
        dxl_controller->setGoalPositionRadian(4, -pp);

        dxl_controller->getPresentPositionRadian(5, &pp);
        dxl_controller->setGoalPositionRadian(6, -pp);
    }
    ros::spin();

    return 0;
}
