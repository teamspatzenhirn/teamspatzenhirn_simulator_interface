/**
 * @file SimCameraCalib.hpp
 * @author jonasotto
 * @date 4/28/21
 * Simulator camera calibration
 *
 * @note File should be reworked once C++ math functions are constexpr.
 */

#ifndef SIMULATORFILTERS_SIMCAMERACALIB_HPP
#define SIMULATORFILTERS_SIMCAMERACALIB_HPP

#include <opencv2/core.hpp>

extern const cv::Size SIM_CAMERA_SIZE;

extern const cv::Matx33f SIM_CAMERA_MATRIX;

extern const cv::Matx<float, 1, 3> SIM_RADIAL_DISTORTION_COEFFICIENTS;
extern const cv::Matx<float, 1, 2> SIM_TANGENTIAL_DISTORTION_COEFFICIENTS;

extern const cv::Matx33f SIM_NEW_CAMERA_MATRIX;
extern const cv::Matx33f SIM_FLOORPLANE_TO_IMAGE_MATRIX;

// relevant for the simulation of the point cloud
constexpr auto SIM_DEPTH_CAMERA_WIDTH = 640 * 2;
constexpr auto SIM_DEPTH_CAMERA_HEIGHT = 480;

constexpr auto SIM_DEPTH_CAMERA_TRANS_X = 0.05F;
constexpr auto SIM_DEPTH_CAMERA_TRANS_Y = 0.0F;
constexpr auto SIM_DEPTH_CAMERA_TRANS_Z = 0.19F;

#endif // SIMULATORFILTERS_SIMCAMERACALIB_HPP
