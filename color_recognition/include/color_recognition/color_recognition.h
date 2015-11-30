
//std
#include <string>

//ros dependencies
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/ColorRGBA.h>
#include <sensor_msgs/image_encodings.h>

const unsigned int IMG_MONO = 0;
const unsigned int IMG_RGB8 = 1;

class ColorRecogNode
{
protected:

    //ros node handle
    ros::NodeHandle nh;

    //image publisher/subscriber
    image_transport::ImageTransport it;
    image_transport::Publisher imagePub;
    image_transport::Subscriber imageSubs;
    cv_bridge::CvImagePtr cvImgPtr;

    // led color publisher
    std_msgs::ColorRGBA colorMsg;
    ros::Publisher colorPub;

    //flag indicating a new image has been received
    bool newImageFlag;

    //image time stamp
    unsigned int tsec;
    unsigned int tnsec;

    //img encoding id
    unsigned int imgEncoding;
    int recog_size;
    int image_height, image_width;
    int start_height, start_width;

protected:
    //callbacks to image subscription
    void imageCallback(const sensor_msgs::Image::ConstPtr &msg);

public:
    //constructor
    ColorRecogNode();

    //destructor
    ~ColorRecogNode();

    //checks if a new image has been received
    bool newImage();

    //execute circle detection with the cureent image
    void process();

    //publish the output image (input image + marked circles)
    void publishImage();

    //publish the circle set data
    void publishColor();
};
