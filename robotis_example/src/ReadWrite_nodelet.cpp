#include <ros/ros.h>
#include <ros/time.h>

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "robotis_example/ReadWrite.h"

namespace robotis_example {

class ReadWriteNodelet : public nodelet::Nodelet {
  public:
    ReadWriteNodelet() {}

    void onInit() {
      ros::NodeHandle node = getNodeHandle();

      example = new ReadWrite(node);
    }

    ~ReadWriteNodelet() {
      if (example) delete example;
    }

  private:
    ReadWrite *example;
};

};

PLUGINLIB_DECLARE_CLASS(robotis_example, ReadWriteNodelet, robotis_example::ReadWriteNodelet, nodelet::Nodelet);
