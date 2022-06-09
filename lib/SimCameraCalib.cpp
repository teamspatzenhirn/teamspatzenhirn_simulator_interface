/**
 * @file SimCameraCalib.cpp
 * @author jonasotto
 * @date 4/28/21
 * Simulator camera calibration
 */

#include "SimCameraCalib.hpp"


// cert-err58-cpp: "Handle all exceptions thrown before main() begins executing"
//  We can't really recover from failing cv::Matx ctor at startup anyways...

// NOLINTNEXTLINE(cert-err58-cpp)
const cv::Size SIM_CAMERA_SIZE{2048, 1536};

const float SIM_ASPECT_RATIO = static_cast<float>(SIM_CAMERA_SIZE.width) / SIM_CAMERA_SIZE.height;

constexpr float SIM_FOV_Y = 1.7;

// NOLINTNEXTLINE(cert-err58-cpp)
const float SIM_FOV_X = 2 * std::atan(SIM_ASPECT_RATIO * std::tan(SIM_FOV_Y / 2));

// NOLINTNEXTLINE(cert-err58-cpp)
const float SIM_FY = SIM_CAMERA_SIZE.height / (2 * std::tan(SIM_FOV_Y / 2));
// NOLINTNEXTLINE(cert-err58-cpp)
const float SIM_FX = SIM_CAMERA_SIZE.width / (2 * std::tan(SIM_FOV_X / 2));

// clang-format off

// NOLINTNEXTLINE(cert-err58-cpp)
const cv::Matx33f SIM_CAMERA_MATRIX{SIM_FY, 0,      SIM_CAMERA_SIZE.width / 2.0F,
                                    0,      SIM_FX, SIM_CAMERA_SIZE.height / 2.0F,
                                    0,      0,      1};

// NOLINTNEXTLINE(cert-err58-cpp)
const cv::Matx<float, 1, 3> SIM_RADIAL_DISTORTION_COEFFICIENTS{0, 0, 0};

// NOLINTNEXTLINE(cert-err58-cpp)
const cv::Matx<float, 1, 2> SIM_TANGENTIAL_DISTORTION_COEFFICIENTS{0, 0};

// Camera matrix of undistorted image
// NOLINTNEXTLINE(cert-err58-cpp)
const cv::Matx33f SIM_NEW_CAMERA_MATRIX = SIM_CAMERA_MATRIX;

// Calculated using calib tool
// NOLINTNEXTLINE(cert-err58-cpp)
const cv::Matx33f SIM_FLOORPLANE_TO_IMAGE_MATRIX{-3.1726,     2.12341,      325.947,
                                                 -2.00932,    -8.69606e-16, -443.848,
                                                 -0.00309868, -9.12748e-19, 0.315518};

// clang-format on
