/**
 * @file simulatorExtrinsic.main.cpp
 * @author jonasotto
 * @date 5/3/21
 * Utility for calculating extrinsic calib from camera position + rotation (available in simulator)
 */


#include <SimulatorFilters/lib/SimCameraCalib.hpp>
#include <opencv2/opencv.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <vector>

std::string vectorToString(tf2::Vector3 v) {
    return "(" + std::to_string(v.x()) + ", " + std::to_string(v.y()) + ", " + std::to_string(v.z()) + ")";
}

double degToRad(double deg) {
    return 2.0 * M_PI * deg / 360;
}

int main() {

    tf2::Transform t;
    t.setOrigin({0.0f, 0.3f, 0.15f}); // Camera position as set in simulator
    tf2::Quaternion q;
    q.setRPY(-M_PI / 2 - degToRad(10), 0, -M_PI / 2); // Camera rotation as set in simulator
    t.setRotation(q);

    cv::Matx33f camMat = SIM_NEW_CAMERA_MATRIX;

    // Default calib points from CalibTool
    std::vector<cv::Point2f> objPts;
    objPts.emplace_back(0.4, -0.4);
    objPts.emplace_back(0.4, 0.4);
    objPts.emplace_back(1.2, -0.4);
    objPts.emplace_back(1.2, 0.4);

    // Corresponding points in image
    std::vector<cv::Point2f> imgPts;

    for (auto &objPt : objPts) {
        tf2::Vector3 objectPoint = {objPt.x, objPt.y, 0};
        std::cout << "Point in spatz coordinates: " << vectorToString(objectPoint) << std::endl;
        auto imagePoint = t.invXform(objectPoint);
        std::cout << "Point in image coordinates: " << vectorToString(imagePoint) << std::endl;
        auto projectedImagePoint = cv::Point3f(imagePoint.x() / imagePoint.z(), imagePoint.y() / imagePoint.z(), 1);
        std::cout << "Point on image plane: " << projectedImagePoint << std::endl;
        auto pixelCoordinate = camMat * projectedImagePoint;
        std::cout << "Pixel coordinates: " << pixelCoordinate << std::endl;
        imgPts.emplace_back(pixelCoordinate.x, pixelCoordinate.y);
        objPt *= 1000; // Legacy code uses millimeters...
    }

    cv::Mat M = cv::getPerspectiveTransform(objPts, imgPts);
    std::cout << "Extrinsic to_img matrix: " << M << std::endl;
}