//
// Created by gilbertorossi on 03.09.18.
//

#ifndef SIMULATOR_FILTERS_SHM_IDS_H
#define SIMULATOR_FILTERS_SHM_IDS_H

#include <opencv2/core.hpp>

#include "HardwareIO/lib/MsgIn.h"
#include "HardwareIO/lib/RCMode.h"

#define SHM_IMAGE_ID 428769
#define SHM_HWIN_ID 428770
#define SHM_HWOUT_ID 428771
#define SHM_DEPTH_ID 428772
#define SHM_VISOUT_ID 428773

#define MAX_IMAGE_WIDTH 2064
#define MAX_IMAGE_HEIGHT 1544

#define MAX_DEPTH_WIDTH (640 * 2)
#define MAX_DEPTH_HEIGHT 480

struct ImageObject {

    unsigned char rgbdata[MAX_IMAGE_WIDTH * MAX_IMAGE_HEIGHT];

    int imageWidth = -1;
    int imageHeight = -1;
};

struct DepthObject {

    float depthPoints[MAX_DEPTH_WIDTH * MAX_DEPTH_HEIGHT * 3];

    int imageWidth = -1;
    int imageHeight = -1;
};

struct HardwareIn {

    double x;
    double y;

    double psi;
    double dPsi;

    double steeringAngle;

    double velX;
    double velY;

    double accX;
    double accY;

    double alphaFront;
    double alphaRear;

    double time;

    double drivenDistance;

    bool paused;

    float laserSensorValue;

    bool binaryLightSensorTriggered;
};

struct HardwareOut {

    double vel;
    double deltaFront, deltaRear;
};

struct VisualizationOut {

    cv::Point2f trajectoryPoints[128];
};

#endif
