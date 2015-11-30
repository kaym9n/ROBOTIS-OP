#include "ball_detector_node.h"

CballDetectorNode::CballDetectorNode() : nh(ros::this_node::getName()) , it(this->nh)
{
      //detector config struct
      detectorConfig detCfg;

      //get user parameters from dynamic reconfigure (yaml entries)
      nh.getParam("gaussian_blur_size", detCfg.gaussian_blur_size); 
      nh.getParam("gaussian_blur_sigma", detCfg.gaussian_blur_sigma); 
      nh.getParam("canny_edge_th", detCfg.canny_edge_th); 
      nh.getParam("hough_accum_resolution", detCfg.hough_accum_resolution);       
      nh.getParam("min_circle_dist", detCfg.min_circle_dist); 
      nh.getParam("hough_accum_th", detCfg.hough_accum_th); 
      nh.getParam("min_radius", detCfg.min_radius);
      nh.getParam("max_radius", detCfg.max_radius); 
      nh.getParam("filter_threshold", detCfg.filter_threshold);
      nh.param<bool>("filter_debug", detCfg.debug, false);
      
      //sets config and prints it
      detector.setParameters(detCfg);
      detector.printConfig();      

      //sets publishers
      imagePub = it.advertise("image_out", 100);
      circlesPub = nh.advertise<btr_ball_detector::circleSetStamped>("circle_set", 100);
      
      //sets subscribers
      imageSubs = it.subscribe("image_in", 1, &CballDetectorNode::imageCallback, this);
      //nh.subscribe("cameraInfo_in", 100, &CballDetectorNode::cameraInfoCallback, this);
            
      //initializes newImageFlag
      newImageFlag = false;
}

CballDetectorNode::~CballDetectorNode()
{
      
}

bool CballDetectorNode::newImage()
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

void CballDetectorNode::process()
{
      if ( cvImgPtrSubs!=NULL )
      {
            //sets input image
            detector.setInputImage(cvImgPtrSubs->image);
            
            // test image filtering
            detector.filterImage(0);

            //detect circles
            detector.houghDetection(this->imgEncoding);
      }
}

void CballDetectorNode::publishImage()
{
      //image_raw topic
      cvImgPub.header.seq ++;
      cvImgPub.header.stamp.sec = this->tsec;
      cvImgPub.header.stamp.nsec = this->tnsec;
      cvImgPub.header.frame_id = "detector"; //To do: get frame_id from input image
      switch(imgEncoding)
      {
            case IMG_RGB8: cvImgPub.encoding = sensor_msgs::image_encodings::RGB8; break;
            case IMG_MONO: cvImgPub.encoding = sensor_msgs::image_encodings::MONO8; break;
            default: cvImgPub.encoding = sensor_msgs::image_encodings::MONO8; break;
      }
      detector.getOutputImage(cvImgPub.image);
      imagePub.publish(cvImgPub.toImageMsg());
}

void CballDetectorNode::publishCircles()
{
      int ii; 
      
      //clears and resize the message
      circlesMsg.circles.clear();
      circlesMsg.circles.resize(detector.howManyCircles());
      
      //fill header
      circlesMsg.header.seq ++;
      circlesMsg.header.stamp.sec = this->tsec;
      circlesMsg.header.stamp.nsec = this->tnsec;      
      circlesMsg.header.frame_id = "detector"; //To do: get frame_id from input image
      
      //fill circle data
      for(ii=0; ii<detector.howManyCircles(); ii++ )
      {
            detector.getCircle(ii, circlesMsg.circles[ii].x, circlesMsg.circles[ii].y, circlesMsg.circles[ii].z);
      }
      
      //publish message
      circlesPub.publish(circlesMsg);
}

void CballDetectorNode::imageCallback(const sensor_msgs::ImageConstPtr & msg)
{
      try
      {
            //std::cout << __LINE__ << ": msg->encoding = " << msg->encoding << std::endl;
            if ( msg->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ) this->imgEncoding = IMG_MONO;
            if ( msg->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0 ) this->imgEncoding = IMG_RGB8;
            this->cvImgPtrSubs = cv_bridge::toCvCopy(msg, msg->encoding);
      }
      catch (cv_bridge::Exception& e)
      {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
      }      
      
      //indicates a new image is available
      this->tsec = msg->header.stamp.sec;
      this->tnsec = msg->header.stamp.nsec;
      this->newImageFlag = true;
      return; 
}

void CballDetectorNode::cameraInfoCallback(const sensor_msgs::CameraInfo & msg)
{
      // nothing to do ...
}
