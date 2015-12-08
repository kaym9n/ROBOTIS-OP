/**
 * @file /include/op_control/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef op_control_QNODE_HPP_
#define op_control_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <std_msgs/Int16.h>
#include <sensor_msgs/JointState.h>
#include <QStringListModel>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace op_control {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

    std::string head_pan_joint_name, head_tilt_joint_name;
    inline double Deg2Rad(int degree) {return (double)degree * M_PI / 180;}
    inline double Rad2Deg(double radian) {return radian * 180 / M_PI;}

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

    void play_motion(int index, std::string name);
    void control_joint(const std::string& joint_name, const double& joint_value);

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();
    void updateHeadPan(double angle);
    void updateHeadTilt(double angle);

private:
	int init_argc;
	char** init_argv;

    ros::Publisher _play_motion_pub;
    ros::Publisher _joint_control_pub;
    ros::Subscriber _joint_sub;
    QStringListModel logging_model;

    void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);
};

}  // namespace op_control

#endif /* op_control_QNODE_HPP_ */
