#include "color_recognition/color_recognition.h"

//node main
int main(int argc, char **argv)
{
      //init ros
      ros::init(argc, argv, "color_recognition_node");

      //create ros wrapper object
      ColorRecogNode color_node;

      //set node loop rate
      ros::Rate _loop_rate(30);

      ROS_INFO("Start color recongnition .....");

      //node loop
      while ( ros::ok() )
      {
            //if new image , do things
            if ( color_node.newImage() )
            {
                  color_node.process();
                  color_node.publishImage();
                  color_node.publishColor();
            }

            //execute pending callbacks
            ros::spinOnce();

            //relax to fit output rate
            _loop_rate.sleep();
      }

      //exit program
      return 0;
}

