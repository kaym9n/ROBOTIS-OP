
// include
#include "op_octomap/op_octomap.h"

//node main
int main(int argc, char **argv)
{
    //init ros
    ros::init(argc, argv, "op_octomap_node");

    //create ros wrapper object
    robotis_op::OPOctoMAP octo_map;


    //set node loop rate
    ros::Rate loop_rate(100);

    //node loop
    while ( ros::ok() )
    {
        octo_map.broadcastTF();

        //execute pending callbacks
        ros::spinOnce();

        //relax to fit output rate
        loop_rate.sleep();
    }

    //exit program
    return 0;
}

