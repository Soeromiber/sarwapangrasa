#include <ros/ros.h>
#include "sarwapangrasa/artag/artag.hpp"
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>

#include <sarwapangrasa/Artag.h>
#include <sarwapangrasa/Artags.h>

cv::Mat cameraMatrix, distCoeffs;
std::string image_topic;

sarwapangrasa::Artags artags;
void cam_cb(const sensor_msgs::Image::ConstPtr& img)
{
  ROS_INFO("Received image");
  sarwapangrasa::artag::ArTag artag(cameraMatrix, distCoeffs, 0.05);
  // artags = artag.detect(img);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "aruco_detector");
  ros::NodeHandle nh;
  ros::Rate rate(30);

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

  nh.getParam("image_topic", image_topic);

  ros::Publisher artagPublisher = nh.advertise<sarwapangrasa::Artags>("artags", 1);
  ros::Subscriber sub = nh.subscribe<sensor_msgs::Image>(image_topic, 1, cam_cb);

  while (ros::ok())
  {
    artagPublisher.publish(artags);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}