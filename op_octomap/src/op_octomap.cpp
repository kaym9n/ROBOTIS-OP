#include <yaml-cpp/yaml.h>
//#include <fstream>

#include "op_octomap/op_octomap.h"

namespace robotis_op {

OPOctoMAP::OPOctoMAP() : nh(), parent_name_("map"), child_name_("base_link")
{
    // has_path = nh.getParam("yaml_path", param_path);

    // if(has_path) std::cout << "Path : " << param_path << std::endl;

    //detector config struct
    //DetectorConfig detCfg;

    /*
    //get user parameters from dynamic reconfigure (yaml entries)
    nh.param<int>("gaussian_blur_size", detCfg.gaussian_blur_size, params_config.gaussian_blur_size);
    if(detCfg.gaussian_blur_size % 2 == 0) detCfg.gaussian_blur_size -= 1;
    if(detCfg.gaussian_blur_size <= 0) detCfg.gaussian_blur_size = 1;
    nh.param<double>("gaussian_blur_sigma", detCfg.gaussian_blur_sigma, params_config.gaussian_blur_sigma);
    nh.param<double>("canny_edge_th", detCfg.canny_edge_th, params_config.canny_edge_th);
    nh.param<double>("hough_accum_resolution", detCfg.hough_accum_resolution, params_config.hough_accum_resolution);
    nh.param<double>("min_circle_dist", detCfg.min_circle_dist, params_config.min_circle_dist);
    nh.param<double>("hough_accum_th", detCfg.hough_accum_th, params_config.hough_accum_th);
    nh.param<int>("min_radius", detCfg.min_radius, params_config.min_radius);
    nh.param<int>("max_radius", detCfg.max_radius, params_config.max_radius);
    nh.param<int>("filter_threshold", detCfg.filter_threshold, params_config.filter_threshold);
    nh.param<bool>("filter_debug", detCfg.debug, params_config.debug);
    */

    //sets publishers
    tf_broadcaster_.reset(new tf::TransformBroadcaster());

    //sets subscribers
    tf_listener_.reset(new tf::TransformListener());
    imu_data_sub_ = nh.subscribe("robot_imu_data", 10, &OPOctoMAP::imuDataCallback, this);

    /*
    // dynamic_reconfigure
    _callback_fnc = boost::bind(&BallDetector::dynParamCallback, this, _1, _2);
    _param_server.setCallback(_callback_fnc);

    //sets config and prints it
    params_config = detCfg;
    init_param = true;
    printConfig();
    */
}

OPOctoMAP::~OPOctoMAP()
{

}

void OPOctoMAP::initRos()
{
    // service
    octo_map_reset_sevice = nh.serviceClient<std_srvs::Empty>("/octomap_server/reset");

    // publisher


    // subscriber


}

void OPOctoMAP::imuDataCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    // orienatation
    current_tf_.transform.rotation.w = msg->orientation.w;
    current_tf_.transform.rotation.x = msg->orientation.x;
    current_tf_.transform.rotation.y = msg->orientation.y;
    current_tf_.transform.rotation.z = msg->orientation.z;

    // translation
	try
	{
    tf::StampedTransform _transform_l_foot, _transform_r_foot;
	tf_listener_->lookupTransform("/l_foot_link", "/base_link", ros::Time(0), _transform_l_foot);
	tf_listener_->lookupTransform("/r_foot_link", "/base_link", ros::Time(0), _transform_r_foot);

    current_tf_.transform.translation.x = (_transform_l_foot.getOrigin().x() + _transform_r_foot.getOrigin().x()) / 2;
    current_tf_.transform.translation.y = (_transform_l_foot.getOrigin().y() + _transform_r_foot.getOrigin().y()) / 2;
    current_tf_.transform.translation.z = (_transform_l_foot.getOrigin().z() + _transform_r_foot.getOrigin().z()) / 2;
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s", ex.what());
		return;
	}
}

void OPOctoMAP::broadcastTF()
{
    current_tf_.header.stamp = ros::Time::now();
    current_tf_.header.frame_id = parent_name_;
    current_tf_.child_frame_id = child_name_;

    tf_broadcaster_->sendTransform(current_tf_);
}

void OPOctoMAP::callOctoMapReset()
{
    std_srvs::Empty _service;
    octo_map_reset_sevice.call(_service);
}

/*
void OPOctoMAP::dynParamCallback(ball_detector::detectorParamsConfig &config, uint32_t level)
{
    params_config.gaussian_blur_size       = config.gaussian_blur_size;
    params_config.gaussian_blur_sigma      = config.gaussian_blur_sigma;
    params_config.canny_edge_th            = config.canny_edge_th;
    params_config.hough_accum_resolution   = config.hough_accum_resolution;
    params_config.min_circle_dist          = config.min_circle_dist;
    params_config.hough_accum_th           = config.hough_accum_th;
    params_config.min_radius               = config.min_radius;
    params_config.max_radius               = config.max_radius;
    params_config.filter_threshold         = config.filter_threshold;
    params_config.debug                    = config.debug_image;

    // gaussian_blur has to be odd number.
    if(params_config.gaussian_blur_size % 2 == 0) params_config.gaussian_blur_size -= 1;
    if(params_config.gaussian_blur_size <= 0) params_config.gaussian_blur_size = 1;

    printConfig();
    saveConfig();
}
*/

void OPOctoMAP::printConfig()
{
    /*
    if(init_param == false) return;

    std::cout << "Detetctor Configuration:" << std::endl
              << "    gaussian_blur_size: " << params_config.gaussian_blur_size << std::endl
              << "    gaussian_blur_sigma: " << params_config.gaussian_blur_sigma << std::endl
              << "    canny_edge_th: " << params_config.canny_edge_th << std::endl
              << "    hough_accum_resolution: " << params_config.hough_accum_resolution << std::endl
              << "    min_circle_dist: " << params_config.min_circle_dist << std::endl
              << "    hough_accum_th: " << params_config.hough_accum_th << std::endl
              << "    min_radius: " << params_config.min_radius << std::endl
              << "    max_radius: " << params_config.max_radius << std::endl
              << "    filter_threshold: " << params_config.filter_threshold << std::endl
              << "    filter_image_to_debug: " << params_config.debug << std::endl << std::endl;
    */
}

void OPOctoMAP::saveConfig()
{
    /*
    if(has_path == false) return;

    YAML::Emitter _out;

    _out << YAML::BeginMap;
    _out << YAML::Key << "gaussian_blur_size"       << YAML::Value << params_config.gaussian_blur_size;
    _out << YAML::Key << "gaussian_blur_sigma"      << YAML::Value << params_config.gaussian_blur_sigma;
    _out << YAML::Key << "canny_edge_th"            << YAML::Value << params_config.canny_edge_th;
    _out << YAML::Key << "hough_accum_resolution"   << YAML::Value << params_config.hough_accum_resolution;
    _out << YAML::Key << "min_circle_dist"          << YAML::Value << params_config.min_circle_dist;
    _out << YAML::Key << "hough_accum_th"           << YAML::Value << params_config.hough_accum_th;
    _out << YAML::Key << "min_radius"               << YAML::Value << params_config.min_radius;
    _out << YAML::Key << "max_radius"               << YAML::Value << params_config.max_radius;
    _out << YAML::Key << "filter_threshold"         << YAML::Value << params_config.filter_threshold;
    _out << YAML::Key << "filter_debug"             << YAML::Value << params_config.debug;
    _out << YAML::EndMap;

    // output to file
    std::ofstream fout(param_path.c_str());
    fout << _out.c_str();
    */
}

}       // namespace robotis_op
