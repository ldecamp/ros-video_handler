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
//Helpers to match histograms between frames and videos
void matchColorHistograms(const Mat& reference, const Mat& target, Mat& dest);
void matchImageWithHistograms(const Mat& img, const Mat& cdfs, Mat& out);
void buildCdfFromPixelIntensity(const Mat& channel, Mat& hist, Mat& cdf);
void buildCdfsFromPixelIntensity(const Mat& image, Mat& cdfs);

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "video_handler");
  ros::NodeHandle _nh("~");

  int source = 0;
  string param = "";

  string topicname = "";
  string colorType = "bgr8";
  int framerate = 30;
  bool resize_frame = false;
  bool play_reversed = false;
  int res_width = 1;
  int res_heigth = 1;
  bool send_Reset = false;
  bool send_Save = false;
  bool hist_matching = false;

  _nh.getParam("source", source);
  _nh.getParam("input", param);
  _nh.getParam("sendReset", send_Reset);
  _nh.getParam("sendSave", send_Save);
  _nh.getParam("matchHistograms", hist_matching);
  _nh.getParam("colorType", colorType);

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
  _image.encoding = colorType;
  // _image.encoding = "bgr8";
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

  bool compute_hist = true;
  Mat cdfs_ref;

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

      //if resize required resize
      if (resize_frame) {
        cv::resize(_image.image, _image.image, newSize);
      }

      if (colorType == "mono8") {
        //convert to gray scale and normalise brightness
        cv::cvtColor(_image.image, _image.image, CV_BGR2GRAY);


        if (hist_matching) {
          if (compute_hist) {
            buildCdfsFromPixelIntensity(_image.image, cdfs_ref);
            compute_hist = false;
          } else {
            matchImageWithHistograms(_image.image, cdfs_ref, _image.image);
          }
        } 
        // else {
        //   cv::equalizeHist(_image.image, _image.image);
        // }

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
    if (source != 0 && send_Save) {
      orb_command.data = "SaveMap";
      _pubCommands.publish(orb_command);
      //sleep 10 sec to give time to ORB to catch up.
      sleep(15);
    }


    //if video finished publish reset to orb command
    if (source != 0 && send_Reset) {
      orb_command.data = "ResetMap";
      _pubCommands.publish(orb_command);
      //sleep 10 sec to give time to ORB to catch up.
      sleep(5);
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

const double epsilon = 0.0001;
const int bins = 256;
void buildCdfFromPixelIntensity(const Mat& channel, Mat& hist, Mat& cdf) {
  hist = Mat::zeros(1, bins, CV_64FC1);
  cdf = Mat::zeros(1, bins, CV_64FC1);
  //Compute histogram from intensity values
  unsigned char *input = (unsigned char*)(channel.data);
  for (int j = 0; j < channel.rows; j++) {
    for (int i = 0; i < channel.cols; i++) {
      int pix_val = static_cast<int>(input[channel.step * j + i]);
      hist.at<double>(pix_val) += 1;
    }
  }
  //Normalize and compute cdf
  double minVal, maxVal;
  minMaxLoc(hist, &minVal, &maxVal);
  cdf.at<double>(0) = hist.at<double>(0);
  for (int j = 1; j < bins; j++) {
    cdf.at<double>(j) = cdf.at<double>(j - 1) + hist.at<double>(j);
  }
  // normalize cdf
  cdf = cdf / cdf.at<double>(bins - 1);
}

void buildCdfsFromPixelIntensity(const Mat& image, Mat& cdfs) {
  vector<Mat> channels;
  cv::split(image, channels);
  std::vector<Mat> cdf_per_channel;
  cdf_per_channel.resize(channels.size());
  //get cdf per channel and merge
  for (unsigned int c = 0; c < channels.size(); c++) {
    Mat hist_chan, cdf_chan;
    buildCdfFromPixelIntensity(channels[c], hist_chan, cdf_chan);
    cdf_per_channel[c] = cdf_chan;
  }
  cv::merge(cdf_per_channel, cdfs);
}

void matchColorHistograms(const Mat& reference, const Mat& target, Mat& dest) {
  vector<Mat> channels_ref;
  cv::split(reference, channels_ref);
  vector<Mat> channels_targ;
  cv::split(target, channels_targ);

  if (channels_ref.size() != channels_targ.size()) {
    cout << "Error: source and target must have same amout of channels" << endl;
    return;
  }

  for (unsigned int c = 0; c < channels_ref.size(); c++) {
    Mat hist_ref, cdf_ref, hist_targ, cdf_targ;
    buildCdfFromPixelIntensity(channels_ref[c], hist_ref, cdf_ref);
    buildCdfFromPixelIntensity(channels_targ[c], hist_targ, cdf_targ);
    Mat Mv(1, bins, CV_8UC1);
    uchar* M = Mv.ptr<uchar>();
    int current = 0;
    //build LUT
    for (int j = 0; j < cdf_targ.cols; j++) {
      double F1t = cdf_targ.at<double>(j);
      for (int k = current; k < cdf_ref.cols; k++) {
        double F2s = cdf_ref.at<double>(k);
        if (abs(F2s - F1t) < epsilon || F2s > F1t) {
          M[j] = k;
          current = k;
          break;
        }
      }
    }
    Mat lut(1, bins, CV_8UC1, M);
    cv::LUT(channels_targ[c], lut, channels_targ[c]);
  }

  cv::merge(channels_targ, dest);
}

void matchImageWithHistograms(const Mat& img, const Mat& cdfs, Mat& out) {
  vector<Mat> channels_cdfs;
  cv::split(cdfs, channels_cdfs);
  vector<Mat> channels_targ;
  cv::split(img, channels_targ);

  if (channels_cdfs.size() != channels_targ.size()) {
    cout << "Error: source and target must have same amout of channels" << endl;
    return;
  }

  for (unsigned int c = 0; c < channels_targ.size(); c++) {
    Mat cdf_ref, hist_targ, cdf_targ;

    cdf_ref = channels_cdfs[c];
    buildCdfFromPixelIntensity(channels_targ[c], hist_targ, cdf_targ);

    Mat Mv(1, bins, CV_8UC1);
    uchar* M = Mv.ptr<uchar>();
    int current = 0;
    //build LUT
    for (int j = 0; j < cdf_targ.cols; j++) {
      double F1t = cdf_targ.at<double>(j);
      for (int k = current; k < cdf_ref.cols; k++) {
        double F2s = cdf_ref.at<double>(k);
        if (abs(F2s - F1t) < epsilon || F2s > F1t) {
          M[j] = k;
          current = k;
          break;
        }
      }
    }
    Mat lut(1, bins, CV_8UC1, M);
    cv::LUT(channels_targ[c], lut, channels_targ[c]);
  }

  cv::merge(channels_targ, out);
}