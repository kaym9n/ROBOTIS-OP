#include "color_recognition/color_recognition.h"

ColorRecogNode::ColorRecogNode() : nh(ros::this_node::getName()) , it(this->nh)
{
    //sets publishers
    imagePub = it.advertise("/image_out", 100);
    colorPub = nh.advertise<std_msgs::ColorRGBA>("/head_led_color", 100);           // for head led
    // colorPub = nh.advertise<std_msgs::ColorRGBA>("eye_led_color", 100);        // for eye leds

    //sets subscribers
    imageSubs = it.subscribe("/image_raw", 1, &ColorRecogNode::imageCallback, this);

    //initializes newImageFlag
    newImageFlag = false;
}

ColorRecogNode::~ColorRecogNode()
{

}

bool ColorRecogNode::newImage()
{
    if ( newImageFlag )
    {
        newImageFlag = false;
        return true;
    }
    else
    {
        return false;
    }
}

void ColorRecogNode::process()
{
    if(this->imgEncoding != IMG_RGB8) return;

    // get average of rgb value
    if ( cvImgPtr != NULL )
    {
        int _sum_r = 0, _sum_g = 0, _sum_b = 0;

        // ROS_INFO("height : %d, width : %d, channels : %d, start_height : %d, start_width : %d", image_height, image_width, RGB_Channel, start_height, start_width);

        for(int _row = start_height; _row < start_height + recog_size; _row++)
        {
            for(int _col = start_width; _col < (start_width + recog_size); _col ++)
            {
                // split rgb channel
                cv::Vec3b _bgr_color = cvImgPtr->image.at<cv::Vec3b>(_row, _col);

                _sum_r += (int) _bgr_color.val[0];
                _sum_g += (int) _bgr_color.val[1];
                _sum_b += (int) _bgr_color.val[2];
            }
        }

        colorMsg.r = _sum_r / (recog_size * recog_size);
        colorMsg.g = _sum_g / (recog_size * recog_size);
        colorMsg.b = _sum_b / (recog_size * recog_size);

        // ROS_INFO("RGB : %f, %f, %f [%d]", colorMsg.r, colorMsg.g, colorMsg.b, recog_size);
    }
}

void ColorRecogNode::publishImage()
{
    int _line_thickness = recog_size * 0.1;
    int _bg_thickness = _line_thickness + 2;

    // draw a rectangle bg(black)
	cv::rectangle(cvImgPtr->image,
					cv::Point(start_width - _line_thickness * 0.5, start_height - _line_thickness * 0.5),
					cv::Point(start_width + recog_size + _line_thickness, start_height + recog_size + _line_thickness),
					cv::Scalar(0, 0, 0),
					_bg_thickness);

    // draw a rectangle
    cv::rectangle(cvImgPtr->image,
                  cv::Point(start_width - _line_thickness * 0.5, start_height - _line_thickness * 0.5),
                  cv::Point(start_width + recog_size + _line_thickness, start_height + recog_size + _line_thickness),
                  cv::Scalar(colorMsg.r, colorMsg.g, colorMsg.b),
                  _line_thickness);

    // convert OpenCV image to ROS image and publish image
    imagePub.publish(cvImgPtr->toImageMsg());
}

void ColorRecogNode::publishColor()
{
    // publish OP_LED message
    colorPub.publish(colorMsg);
}

void ColorRecogNode::imageCallback(const sensor_msgs::Image::ConstPtr & msg)
{
    // convert ROS image to OpenCV image
    try
    {
        if ( msg->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ) this->imgEncoding = IMG_MONO;
        if ( msg->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0 ) this->imgEncoding = IMG_RGB8;
        this->cvImgPtr = cv_bridge::toCvCopy(msg, msg->encoding);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    image_height = msg->height;
    image_width = msg->width;
    recog_size = (int)(image_height * 0.1);
    start_height = (int)((image_height - recog_size) * 0.5);
    start_width = (int)((image_width - recog_size) * 0.5);

    // set new image flag
    this->newImageFlag = true;

    return;
}
