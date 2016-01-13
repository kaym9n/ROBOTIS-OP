#ifndef _OP_OCTOMAP_H
#define _OP_OCTOMAP_H

//std
#include <string>

//ros dependencies
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>

namespace robotis_op {

class OPOctoMAP
{

    // Variable
    protected:
        //ros node handle
        ros::NodeHandle nh;

        //publisher/subscriber        
        ros::Subscriber imu_data_sub_;
        boost::shared_ptr<tf::TransformBroadcaster> tf_broadcaster_;
        boost::shared_ptr<tf::TransformListener> tf_listener_;

        //service
        ros::ServiceClient octo_map_reset_sevice;

        geometry_msgs::TransformStamped current_tf_;
        std::string parent_name_, child_name_;
        /*
        //dynamic reconfigure
        DetectorConfig params_config;
        std::string param_path;
        bool has_path;

        dynamic_reconfigure::Server<ball_detector::detectorParamsConfig> _param_server;
        dynamic_reconfigure::Server<ball_detector::detectorParamsConfig>::CallbackType _callback_fnc;
        */

    public:

    // Function
    protected:
        void initRos();
        void callOctoMapReset();
        void imuDataCallback(const sensor_msgs::Imu::ConstPtr &msg);
        /*
        void dynParamCallback(ball_detector::detectorParamsConfig &config, uint32_t level);
        */

    public:
        //constructor
        OPOctoMAP();
        //destructor
        ~OPOctoMAP();

        void broadcastTF();
        void printConfig();
        void saveConfig();

};

}       // namespace
#endif  // _OP_OCTOMAP_H
