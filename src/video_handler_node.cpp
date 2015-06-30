#include <stdio.h>
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <opencv/cv.h>
// #include <termios.h>
// #include <unistd.h>
// #include <fcntl.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <boost/algorithm/string.hpp>

#include <sqlite3.h>

#include "std_msgs/String.h"

using namespace std;
using namespace cv;
using namespace boost;

// int kbhit(void);
vector<string> loadFromDatabase(const char* dbPath);

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "video_handler");
  ros::NodeHandle _nh("~");

  int source = 0;
  string param = "";

  string topicname = "";
  int framerate = 30;
  bool resize_frame = false;
  bool play_reversed = false;
  int res_width = 1;
  int res_heigth = 1;
  bool send_Reset = false;

  _nh.getParam("source", source);
  _nh.getParam("input", param);
  _nh.getParam("sendReset", send_Reset);

  _nh.getParam("framerate", framerate);
  _nh.getParam("topicname", topicname);

  _nh.getParam("resize", resize_frame);
  _nh.getParam("newWidth", res_width);
  _nh.getParam("newHeigth", res_heigth);

  // _nh.getParam("play_reversed", play_reversed);

  Size newSize(res_width, res_heigth);

  image_transport::ImageTransport _it(_nh);
  image_transport::Publisher _pub = _it.advertise(topicname, 10, true);
  ros::Publisher _pubCommands = _nh.advertise<std_msgs::String>("/orb_slam/commands", 1, true);
  ROS_INFO("... topic published.");


  sensor_msgs::ImagePtr _publishImage;
  cv_bridge::CvImage _image;
  _image.encoding = "bgr8";
  std_msgs::String orb_command;

  ros::Rate _looprate(framerate);
  VideoCapture _camera;
  vector<string> inputs;

  if (source == 0 || source == 1) {
    inputs.push_back(param);
  } else {
    //load input from database
    inputs = loadFromDatabase(param.c_str());
  }

  cout << "items to play: " << inputs.size() << endl;

  for (unsigned int v = 0; v < inputs.size(); ++v)
  {
    string current = inputs[v];
    if (source == 0) {
      cout << "streaming from device: /dev/video" << current << endl;
      _camera.open(atoi(current.c_str()));
    } else {
      cout << "streaming from file: " << current << endl;
      _camera.open(current);
    }

    //if not 0 publish message to orb to initialize new video
    if (source != 0) {
      vector<string> result;
      split( result, current, is_any_of("/."), token_compress_on );
      //Build message
      string  filename = result[result.size() - 2];
      orb_command.data = "SetVideoKey:" + filename;
      _pubCommands.publish(orb_command);
    }

    //read video
    if (!_camera.isOpened()) {
      ROS_ERROR("Unable to connect to video source.");
      continue;
    }
    ROS_INFO("... connected to video source.");


    while (_camera.read(_image.image)) {
      //if shutdown requested exit
      if (!ros::ok()) {
        ros::shutdown();
        return 0;
      }

      // if (kbhit()) {
      //   int c = getchar();   //check if database need to be saved.
      //   if (c == 'n') {
      //     cout << "Skipping to next video" << endl;
      //     break;
      //   }
      // }
      //if resize required resize
      if (resize_frame) {
        cv::resize(_image.image, _image.image, newSize);
      }

      //publish image
      _publishImage = _image.toImageMsg();
      if (source != 0) {
        _publishImage->header.stamp = ros::Time(_camera.get(CV_CAP_PROP_POS_MSEC) / 1000);
      }
      _pub.publish(_publishImage);
      _looprate.sleep();
      ros::spinOnce();
    }



    //if video finished publish reset to orb command
    if (source != 0 && send_Reset) {
      orb_command.data = "SaveAndResetMap";
      _pubCommands.publish(orb_command);
      //sleep 10 sec to give time to ORB to catch up.
      sleep(10);
    }
  }

  ros::shutdown();
  return 0;
}

std::vector<string> loadFromDatabase(const char* dbPath) {
  std::vector<string> result;
  cout << "loading from " << dbPath << endl;
  sqlite3* db;
  if (sqlite3_open(dbPath, &db) != SQLITE_OK) {
    cout << "Could not load database." << endl;
    sqlite3_close(db);
    return result;
  }


  sqlite3_stmt *statement;
  string query = "select path from Routes";

  if (sqlite3_prepare_v2(db, query.c_str(), -1, &statement, 0 ) == SQLITE_OK )
  {
    int res = 0;

    while ( 1 )
    {
      res = sqlite3_step(statement);
      if ( res == SQLITE_ROW )
      {
        string s = (char*)sqlite3_column_text(statement, 0);
        result.push_back(s);
      }

      if ( res == SQLITE_DONE || res == SQLITE_ERROR)
      {
        break;
      }
    }
  } else {
    cout << "problem here " << sqlite3_errmsg(db) << endl;
  }
  sqlite3_close(db);
  return result;
}


// //Helper to detect if a key has been presses
// int kbhit(void)
// {
//   struct termios oldt, newt;
//   int ch;
//   int oldf;

//   tcgetattr(STDIN_FILENO, &oldt);
//   newt = oldt;
//   newt.c_lflag &= ~(ICANON | ECHO);
//   tcsetattr(STDIN_FILENO, TCSANOW, &newt);
//   oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
//   fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

//   ch = getchar();

//   tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
//   fcntl(STDIN_FILENO, F_SETFL, oldf);

//   if (ch != EOF)
//   {
//     ungetc(ch, stdin);
//     return 1;
//   }

//   return 0;
// }