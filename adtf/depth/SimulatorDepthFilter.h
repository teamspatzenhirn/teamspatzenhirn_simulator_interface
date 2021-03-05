#ifndef SIMULATOR_IMAGE_FILTER_H
#define SIMULATOR_IMAGE_FILTER_H

#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>

#include <opencv2/opencv.hpp>

#include "ADTF_Helper/adtf/Filter.h"
#include "ADTF_Helper/adtf/Logging.h"
#include "DummyDevice.h"
#include "Environment/adtf/adtf_type.h"
#include "ImageProcessing/adtf/adtf_types.h"
#include "RealsenseCaptureDevice/adtf/FusionFilter/adtf_type.h"
#include "SimulatorFilters/lib/shm_ids.h"
#include "SimulatorFilters/lib/shmcomm.h"
#include "spatzversion.h"

#define OID_ADTF_SIMU_CAPTURE_FILTER "adtf.SimulatorDepthInput"

class SimulatorDepthFilter : public adtf_helper::Filter<adtf::cBaseIODeviceFilter> {

    ADTF_FILTER(OID_ADTF_SIMU_CAPTURE_FILTER, "Simulator Depth Input", adtf::OBJCAT_SensorDevice)

  protected:
    static constexpr auto DEPTH_WIDTH = 640 * 2;
    static constexpr auto DEPTH_HEIGHT = 480;
    static constexpr auto DEPTH_LEN = DEPTH_HEIGHT * DEPTH_WIDTH;

    Pin_ptr<adtf_types::SpatzPointcloudMsg> pointcloudOut = {this, "PointCloudOut", PinType::Output};
    Pin_ptr<adtf_types::SpatzPointcloudMsg> filteredPointcloudOut = {this, "FilteredPointCloudOut", PinType::Output};

    SimulatorSHM::SHMComm<DepthObject> depthRx;
    SimulatorSHM::SHMComm<HardwareIn> spatzRx;

    int failCounter;
    bool prevBinaryLightSensorTriggered;


  public:
    SimulatorDepthFilter(const tChar *__info);
    virtual ~SimulatorDepthFilter();

  protected:
    virtual tResult Init(tInitStage eStage, __exception = NULL) override;
    virtual tResult Shutdown(tInitStage eStage, __exception = NULL) override;
    virtual tResult Start(__exception = NULL) override;
    virtual tResult Stop(__exception = NULL) override;
    virtual tResult OnCreatePins() override;
    virtual tResult OpenDevice(__exception);
    virtual tResult ReadAndTransmitData() override;
};

#endif
