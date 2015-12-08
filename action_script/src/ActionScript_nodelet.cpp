#include <unistd.h>
#include <libgen.h>
#include <ros/ros.h>
#include <ros/time.h>

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "action_script/ActionScript.h"

namespace ROBOTIS {

std::string file_name = "/home/robotis/catkin_ws/src/ROBOTIS-OP/action_script/Data/motion_4096.bin";

//void change_current_dir()
//{
//    char exepath[1024] = {0};
//    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
//    {
//        if(chdir(dirname(exepath)))
//            fprintf(stderr, "chdir error!! \n");
//    }
//}

class ActionScriptNodelet : public nodelet::Nodelet {
  public:
    ActionScriptNodelet() {}

    void onInit() {
      ros::NodeHandle node = getNodeHandle();
      ros::NodeHandle param = getPrivateNodeHandle();

      action_scrpit = new ActionScript(node, param);

      char _file_name[128];

//      if(argc < 2)
              strcpy(_file_name, file_name.c_str()); // Set default motion file path
//          else
//              strcpy(_file_name, argv[1]);

      // _file_name = file_name.c_str();

      if(action_scrpit->LoadFile(_file_name) == true)
          action_scrpit->Process();
      else
          return;
    }

    ~ActionScriptNodelet() {
      if (action_scrpit) delete action_scrpit;
    }

  private:
    ActionScript *action_scrpit;
};

}   // namespace

PLUGINLIB_DECLARE_CLASS(action_script, ActionScriptNodelet, ROBOTIS::ActionScriptNodelet, nodelet::Nodelet);
