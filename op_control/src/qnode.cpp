/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <sstream>
#include "../include/op_control/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace op_control {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
    init_argc(argc),
    init_argv(argv)
{}

QNode::~QNode()
{
    if(ros::isStarted()) {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();
}

bool QNode::init()
{
    ros::init(init_argc,init_argv,"op_control");
    if ( ! ros::master::check() ) {
        return false;
    }

    ros::start(); // explicitly needed since our nodehandle is going out of scope.

    head_pan_joint_name = "head_pan";
    head_tilt_joint_name = "head_tilt";

    ros::NodeHandle _nh;
    // Add your ros communications here.
    _play_motion_pub    = _nh.advertise<std_msgs::Int16>("play_motion", 1000);
    _joint_control_pub  = _nh.advertise<sensor_msgs::JointState>("controller_joint_states", 1000);
    _joint_sub          = _nh.subscribe("op2_joint_states", 1, &QNode::jointStatesCallback, this);

    start();
    return true;
}

void QNode::run()
{
    ros::Rate loop_rate(1);

    while ( ros::ok() )
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
    logging_model.insertRows(logging_model.rowCount(),1);
    std::stringstream logging_model_msg;
    switch ( level ) {
        case(Debug) : {
            ROS_DEBUG_STREAM(msg);
            logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
            break;
        }
        case(Info) : {
            ROS_INFO_STREAM(msg);
            logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
            break;
        }
        case(Warn) : {
            ROS_WARN_STREAM(msg);
            logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
            break;
        }
        case(Error) : {
            ROS_ERROR_STREAM(msg);
            logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
            break;
        }
        case(Fatal) : {
            ROS_FATAL_STREAM(msg);
            logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
            break;
        }
    }
    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::play_motion(int index, std::string name)
{
    std_msgs::Int16 _motion_data;
    _motion_data.data = index;

    _play_motion_pub.publish(_motion_data);

    std::stringstream _ss;
    _ss << "Play Motion #" << index << " : " << name;
    log(Info, _ss.str());
}

void QNode::control_joint(const std::string& joint_name, const double& joint_value)
{
    sensor_msgs::JointState _joint;
    _joint.header.stamp = ros::Time::now();
    _joint.name.push_back(joint_name);
    _joint.position.push_back(joint_value);

    _joint_control_pub.publish(_joint);
}

void QNode::jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    if(msg->name.size() == 0) return;

    for(int ix = 0; ix < msg->name.size(); ix++)
    {
        if(msg->name[ix] == head_pan_joint_name)
        {
            updateHeadPan(Rad2Deg(msg->position[ix]));
        }
        else if(msg->name[ix] == head_tilt_joint_name)
        {
            updateHeadTilt(Rad2Deg(msg->position[ix]));
        }
    }
}

}  // namespace op_control
