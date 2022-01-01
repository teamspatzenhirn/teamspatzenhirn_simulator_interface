/**
 * @file SimCameraCalib.hpp
 * @author jonasotto
 * @date 4/28/21
 * Simulator camera calibration
 */

#ifndef SIMULATORFILTERS_SIMCAMERACALIB_HPP
#define SIMULATORFILTERS_SIMCAMERACALIB_HPP

#include <opencv2/core.hpp>


// cert-err58-cpp: "Handle all exceptions thrown before main() begins executing"
//  We cant really recover from failing cv::Matx ctor at startup anyways...

// NOLINTNEXTLINE(cert-err58-cpp)
const inline cv::Size SIM_CAMERA_SIZE{2048, 1536};

const inline float SIM_ASPECT_RATIO = static_cast<float>(SIM_CAMERA_SIZE.width) / SIM_CAMERA_SIZE.height;

const inline float SIM_FOV_Y = (M_PI * 0.5F);
// NOLINTNEXTLINE(cert-err58-cpp)
const inline float SIM_FOV_X = 2 * std::atan(SIM_ASPECT_RATIO * std::tan(SIM_FOV_Y / 2));

// NOLINTNEXTLINE(cert-err58-cpp)
const inline float SIM_FY = SIM_CAMERA_SIZE.height / (2 * std::tan(SIM_FOV_Y / 2));
// NOLINTNEXTLINE(cert-err58-cpp)
const inline float SIM_FX = SIM_CAMERA_SIZE.width / (2 * std::tan(SIM_FOV_X / 2));

// clang-format off

// NOLINTNEXTLINE(cert-err58-cpp)
const inline cv::Matx33f SIM_CAMERA_MATRIX{SIM_FY, 0,      SIM_CAMERA_SIZE.width / 2.0F,
                                           0,      SIM_FX, SIM_CAMERA_SIZE.height / 2.0F,
                                           0,      0,      1};

// NOLINTNEXTLINE(cert-err58-cpp)
const inline cv::Matx<float, 1, 3> SIM_RADIAL_DISTORTION_COEFFICIENTS{0, 0, 0};

// NOLINTNEXTLINE(cert-err58-cpp)
const inline cv::Matx<float, 1, 2> SIM_TANGENTIAL_DISTORTION_COEFFICIENTS{0, 0};

// Camera matrix of undistorted image
// NOLINTNEXTLINE(cert-err58-cpp)
const inline cv::Matx33f SIM_NEW_CAMERA_MATRIX = SIM_CAMERA_MATRIX;

// Calculated using simulatorExtrinsic tool from cam position (0.13, 0, 0.24) and 17° angle
// NOLINTNEXTLINE(cert-err58-cpp)
const inline cv::Matx33f SIM_FLOORPLANE_TO_IMAGE_MATRIX{-18.0840047303783,    14.18272333593642, 1023.999424911321,
                                                        -9.416378271252997,   0,                 -3026.181619229515,
                                                        -0.01766016133751843, 0,                 1};
// relevant for the simulation of the point cloud
constexpr auto SIM_DEPTH_CAMERA_WIDTH = 640 * 2;
constexpr auto SIM_DEPTH_CAMERA_HEIGHT = 480;

constexpr auto SIM_DEPTH_CAMERA_TRANS_X = 0.05F;
constexpr auto SIM_DEPTH_CAMERA_TRANS_Y = 0.0F;
constexpr auto SIM_DEPTH_CAMERA_TRANS_Z = 0.19F;

// clang-format on

#endif // SIMULATORFILTERS_SIMCAMERACALIB_HPP
