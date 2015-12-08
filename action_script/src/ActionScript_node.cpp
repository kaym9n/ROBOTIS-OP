#include <unistd.h>
#include <libgen.h>

#include "action_script/ActionScript.h"

std::string file_name = "/home/robotis/catkin_ws/src/ROBOTIS-OP/action_script/Data/motion_4096.bin";

void change_current_dir()
{
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
    {
        if(chdir(dirname(exepath)))
            fprintf(stderr, "chdir error!! \n");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "action_script");
    ROBOTIS::ActionScript action(ros::NodeHandle(), ros::NodeHandle("~"));

    char _file_name[128];

    if(argc < 2)
            strcpy(_file_name, file_name.c_str()); // Set default motion file path
        else
            strcpy(_file_name, argv[1]);

    // _file_name = file_name.c_str();

    if(action.LoadFile(_file_name) == true)
        action.Process();
    else
    	return 0;

	ROS_INFO("here?");
    ros::spin();

    return 0;
}
