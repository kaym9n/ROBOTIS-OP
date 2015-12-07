#include "action_script/ActionScript.h"

std::string file_name = "../Data/motion_4096.bin";
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

    if(action.LoadFile(_file_name))
        action.Process();

    ros::spin();

    return 0;
}
