#include "sarwapangrasa/artag/artag.hpp"

namespace sarwapangrasa
{
    namespace artag
    {
        ArTag::ArTag(){}; // Default constructor

        ArTag::ArTag(cv::Mat cameraMatrix, cv::Mat distCoeffs, double markerSize, int dictionaryId, double centerThreshold, double perpendicularThreshold)
        {
            this->cameraMatrix = cameraMatrix;
            this->distCoeffs = distCoeffs;
            // std::vector<double>& mSize
            // this->markerSizes = *mSize;
            this->markerSize = markerSize;
            this->dictionary = cv::aruco::getPredefinedDictionary(dictionaryId);
            this->detectorParams = cv::aruco::DetectorParameters::create();
            this->detectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
            this->detectorParams->cornerRefinementWinSize = 5;  
            this->detectorParams->cornerRefinementMaxIterations = 30;
            this->detectorParams->cornerRefinementMinAccuracy = 0.1;
            this->detectorParams->errorCorrectionRate = 0.6;
            this->detectorParams->minMarkerPerimeterRate = 0.03;
            this->detectorParams->maxMarkerPerimeterRate = 4.0;
            this->detectorParams->polygonalApproxAccuracyRate = 0.03;
            this->detectorParams->minCornerDistanceRate = 0.05;
            this->detectorParams->minDistanceToBorder = 3;
            this->detectorParams->minMarkerDistanceRate = 0.05;
            this->detectorParams->adaptiveThreshWinSizeMin = 3;
            this->detectorParams->adaptiveThreshWinSizeMax = 23;
            this->detectorParams->adaptiveThreshWinSizeStep = 10;
            this->detectorParams->adaptiveThreshConstant = 7;
            this->detectorParams->minOtsuStdDev = 5.0;
            this->detectorParams->perspectiveRemovePixelPerCell = 4;
            this->detectorParams->perspectiveRemoveIgnoredMarginPerCell = 0.13;
            this->detectorParams->maxErroneousBitsInBorderRate = 0.35;
            this->detectorParams->minOtsuStdDev = 5.0;
            this->detectorParams->errorCorrectionRate = 0.6;
            this->centerThreshold = centerThreshold;
            this->perpendicularThreshold = perpendicularThreshold;
        }
        
        Artags ArTag::detect(const sensor_msgs::Image::ConstPtr& img)
        {
            Artags artags;
            cv::Mat image = cv_bridge::toCvShare(img, "bgr8")->image;
            cv::Mat gray;
            cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
            std::vector<int> markerIds;
            std::vector<std::vector<cv::Point2f> > markerCorners, rejectedCandidates;
            cv::aruco::detectMarkers(gray, dictionary, markerCorners, markerIds, detectorParams, rejectedCandidates);

            if(markerIds.size() == 0)
            {
                return artags;
            }

            artags.Size = markerIds.size();
            
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(markerCorners, this->markerSize, this->cameraMatrix, this->distCoeffs, rvecs, tvecs);

            for (int i = 0; i < markerIds.size(); i++)
            {
                Artag artag;
                artag.id = markerIds[i];
                
                cv::Vec3d rvec, tvec;
                rvec = rvecs[i];
                tvec = tvecs[i];
                cv::Mat rotMat;
                cv::Rodrigues(rvec, rotMat);

                // Position
                cv::Mat posMat = -rotMat.t() * tvec;
                cv::Vec3d position = cv::Vec<double, 3>(posMat.ptr<double>());

                // Rotation in quaternion
                cv::Matx33d rotation;
                cv::Rodrigues(rvec, rotation);
                cv::Matx33d rot_mat(rotation);
                double roll, pitch, yaw;
                roll = atan2(rot_mat(2, 1), rot_mat(2, 2));
                pitch = atan2(-rot_mat(2, 0), sqrt(rot_mat(2, 1)*rot_mat(2, 1) + rot_mat(2, 2)*rot_mat(2, 2)));
                yaw = atan2(rot_mat(1, 0), rot_mat(0, 0));

                std::array<double, 3> euler = {roll, pitch, yaw};
                ganisakta::quaternion::Quaternion q(euler);

                artag.position.x = position[0];
                artag.position.y = position[1];
                artag.position.z = position[2];
                artag.orientation.x = q.x;
                artag.orientation.y = q.y;
                artag.orientation.z = q.z;
                artag.orientation.w = q.w;
                artag.rotation.x = roll;
                artag.rotation.y = pitch;
                artag.rotation.z = yaw;

                bool isCentered = false;
                cv::Point2f center = cv::Point2f(image.cols / 2, image.rows / 2);

                for (int j = 0; j < markerCorners[i].size(); j++)
                {
                    if (markerCorners[i][j].x < center.x + 10 && markerCorners[i][j].x > center.x - 10 && markerCorners[i][j].y < center.y + 10 && markerCorners[i][j].y > center.y - 10)
                    {
                        isCentered = true;
                        break;
                    }
                }

                bool isPerpendicular = false;
                if (abs(roll) < 0.1 && abs(pitch) < 0.1)
                {
                    isPerpendicular = true;
                }

                artag.isCentered = isCentered;
                artag.isPerpendicular = isPerpendicular;

                artags.Artags.push_back(artag);
            }
            
            return artags;
        }
    }
}