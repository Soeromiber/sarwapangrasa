#pragma once

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <sensor_msgs/Image.h>

#include <sarwapangrasa/Artag.h>
#include <sarwapangrasa/Artags.h>
#include <ganisakta/quaternion/quaternion.hpp>

namespace sarwapangrasa
{
    namespace artag
    {
        class ArTag
        {
            public:
                ArTag(cv::Mat cameraMatrix, cv::Mat distCoeffs, double markerSize, int dictionaryId = 10, double centerThreshold = 10, double perpendicularThreshold = 0.1);

                Artags detect(const sensor_msgs::Image::ConstPtr& img);
            private:
                cv::Ptr<cv::aruco::DetectorParameters> detectorParams;
                cv::Ptr<cv::aruco::Dictionary> dictionary;
                cv::Mat cameraMatrix, distCoeffs;
                double markerSize = 0.05;
                double centerThreshold = 10;
                double perpendicularThreshold = 0.1;
        };
    }
}