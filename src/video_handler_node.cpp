#include <stdio.h>
#include <ros/ros.h>
#include <iostream>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <opencv/cv.h>
#include "std_msgs/String.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

using namespace std;
using namespace cv;

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "video_handler");
  ros::NodeHandle _nh("~");

  bool stream_from_file = false;
  int device_id = 0;
  string filepath = "";
  string topicname = "";
  int framerate = 30;

  _nh.getParam("stream_from_file", stream_from_file);
  _nh.getParam("device_id", device_id);
  _nh.getParam("filepath", filepath);
  _nh.getParam("framerate", framerate);
  _nh.getParam("topicname", topicname);

  image_transport::ImageTransport _it(_nh);
  image_transport::Publisher _pub = _it.advertise("/camera/image_raw", 1);
  ROS_INFO("... topic published.");


  sensor_msgs::ImagePtr _publishImage;
  cv_bridge::CvImage _image;
  ros::Rate _looprate(framerate);

  VideoCapture *_camera;


  if (stream_from_file) {
    cout << "streaming from file: " << filepath << endl;
    _camera = new VideoCapture(filepath);
  } else {
    cout << "streaming from device: /dev/video" << device_id << endl;
    _camera = new VideoCapture(device_id);
  }

  if (!_camera->isOpened()) {
    ROS_ERROR("Unable to connect to camera");
    ros::shutdown();
  }
  ROS_INFO("... connected to video source.");

  _image.encoding = "bgr8";

  Size resized(1920 / 2, 1080 / 2);

  while (ros::ok())
  {
    *_camera >> _image.image;
    cv::resize(_image.image, _image.image, resized);
    _publishImage = _image.toImageMsg();
    _pub.publish(_publishImage);
    _looprate.sleep();
    ros::spinOnce();
  }
  return 0;
}

