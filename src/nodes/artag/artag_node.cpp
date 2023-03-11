#include <ros/ros.h>
#include "sarwapangrasa/artag/artag.hpp"
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <string>

#include <sarwapangrasa/Artag.h>
#include <sarwapangrasa/Artags.h>

cv::Mat cameraMatrix, distCoeffs;
std::string image_topic;
sarwapangrasa::artag::ArTag artag;
std::vector<double> markerSizes;
double markerSize;

sarwapangrasa::Artags artags;
void cam_cb(const sensor_msgs::Image::ConstPtr& img)
{
  ROS_INFO("Received image");
  artags = artag.detect(img);
}

void handleOverride(std::vector<double>& msize,std::string& override)
{
  std::stringstream ss(override);
  std::string range_value_pair;
  while (std::getline(ss, range_value_pair, ';')) {
      // process each range-value pair
      std::stringstream range_value_stream(range_value_pair);
      std::string range_str, value_str;
      std::getline(range_value_stream, range_str, ':');
      std::getline(range_value_stream, value_str, ':');
      double value = stod(value_str);
      std::stringstream range_stream(range_str);
      std::string index_str;
      while (std::getline(range_stream, index_str, ',')) {
          // process each index in the range
          std::stringstream index_stream(index_str);
          int start, end;
          char dash;
          if (index_stream >> start) {
              end = start;
              if (index_stream >> dash >> end) {
                  // range in format "start-end"
                  for (int i = start; i <= end; i++) {
                      msize[i] = value;
                  }
              } else {
                  // single index
                  msize[start] = value;
              }
          }
      }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "aruco_detector");
  ros::NodeHandle nh;
  ros::Rate rate(30);

  // Check for yaml files
  XmlRpc::XmlRpcValue markerSizesYaml;
  if (nh.getParam("/artag_node/marker_sizes", markerSizesYaml)) {
    double defaultValue = markerSizesYaml["default"];
    markerSizes = std::vector<double>(100, defaultValue);
    handleOverride(markerSizes, markerSizesYaml["override"]);
    markerSize = markerSizesYaml["default"];
  }

  // Read camera matrix from config file
  XmlRpc::XmlRpcValue cameraMatrixList;
  if (nh.getParam("/artag_node/camera_matrix", cameraMatrixList)) {
      int rows = cameraMatrixList["rows"];
      int cols = cameraMatrixList["cols"];
      cv::Matx<double, 3, 3> cameraMatrixVals;
      for (int i = 0; i < rows; i++) {
          for (int j = 0; j < cols; j++) {
              cameraMatrixVals(i, j) = cameraMatrixList["data"][i*cols+j];
          }
      }
      cameraMatrix = cv::Mat(cameraMatrixVals);
  }

  // Read distortion coefficients from config file
  XmlRpc::XmlRpcValue distortionCoefficientsList;
  if (nh.getParam("/artag_node/distortion_coefficients", distortionCoefficientsList)) {
      int rows = distortionCoefficientsList["rows"];
      int cols = distortionCoefficientsList["cols"];
      cv::Matx<double, 1, 5> distortionCoefficientsVals;
      for (int i = 0; i < rows; i++) {
          for (int j = 0; j < cols; j++) {
              distortionCoefficientsVals(i, j) = distortionCoefficientsList["data"][i*cols+j];
          }
      }
      distCoeffs = cv::Mat(distortionCoefficientsVals);
  }

  nh.getParam("/artag_node/image_topic", image_topic);

  artag = sarwapangrasa::artag::ArTag(cameraMatrix, distCoeffs, markerSize);
  ros::Publisher artagPublisher = nh.advertise<sarwapangrasa::Artags>("artags", 1);
  ros::Subscriber sub = nh.subscribe<sensor_msgs::Image>(image_topic.c_str(), 1, cam_cb);

  while (ros::ok())
  {
    artagPublisher.publish(artags);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}