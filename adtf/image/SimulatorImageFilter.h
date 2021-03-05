#ifndef SIMULATOR_IMAGE_FILTER_H
#define SIMULATOR_IMAGE_FILTER_H

#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>

#include <opencv2/opencv.hpp>

#include "ADTF_Helper/adtf/Filter.h"
#include "ADTF_Helper/adtf/Logging.h"
#include "DummyDevice.h"
#include "FlirCaptureDevice/adtf/adtf_type.h"
#include "ImageProcessing/lib/cameraParams.h"
#include "SimulatorFilters/lib/shm_ids.h"
#include "SimulatorFilters/lib/shmcomm.h"
#include "spatzversion.h"

#define OID_ADTF_SIMU_CAPTURE_FILTER "adtf.SimulatorImageInput"

class SimulatorImageFilter : public adtf_helper::Filter<adtf::cBaseIODeviceFilter> {

    ADTF_FILTER(OID_ADTF_SIMU_CAPTURE_FILTER, "Simulator Image Input", adtf::OBJCAT_SensorDevice)

  protected:
    Pin_ptr<adtf_types::GrayPixel> m_oColorOutputPin;

    camera::cameraParam smartekCamParam;

    SimulatorSHM::SHMComm<ImageObject> rx;

    int failCounter;

  public:
    SimulatorImageFilter(const tChar *__info);
    virtual ~SimulatorImageFilter();

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
