#include "SimulatorImageFilter.h"

#include <SimulatorFilters/lib/SimCameraCalib.hpp>

ADTF_FILTER_PLUGIN("Simulator Image Input", OID_ADTF_SIMU_CAPTURE_FILTER, SimulatorImageFilter)

SimulatorImageFilter::SimulatorImageFilter(const tChar *__info) :
    Filter<cBaseIODeviceFilter>(__info),
    rx(SimulatorSHM::CLIENT, SHM_IMAGE_ID) {

    m_pDevice = new DummyDevice();
}

SimulatorImageFilter::~SimulatorImageFilter() {
}

tResult SimulatorImageFilter::OpenDevice(__exception) {
    RETURN_NOERROR;
}

tResult SimulatorImageFilter::Init(tInitStage eStage, __exception) {

    RETURN_IF_FAILED(Filter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst) {
        smartekCamParam.size = SIM_CAMERA_SIZE;
        smartekCamParam.camMat = SIM_CAMERA_MATRIX;

        smartekCamParam.distTanMat = SIM_TANGENTIAL_DISTORTION_COEFFICIENTS;
        smartekCamParam.distRadMat = SIM_RADIAL_DISTORTION_COEFFICIENTS;
        smartekCamParam.roiMat = SIM_NEW_CAMERA_MATRIX;
        smartekCamParam.toImg = SIM_FLOORPLANE_TO_IMAGE_MATRIX;
        smartekCamParam.toCar = smartekCamParam.toImg.inv();

        smartekCamParam.undistNeeded = false;
        smartekCamParam.isSet = true;

    } else if (eStage == StageNormal) {

    } else if (eStage == StageGraphReady) {
        rx.attach();
        failCounter = 0;
    }

    RETURN_NOERROR;
}

tResult SimulatorImageFilter::Shutdown(tInitStage eStage, __exception) {

    RETURN_IF_FAILED(Filter::Shutdown(eStage, __exception_ptr));

    if (eStage == StageFirst) {

    } else if (eStage == StageGraphReady) {
        rx.detach();
    }

    RETURN_NOERROR;
}

tResult SimulatorImageFilter::Start(__exception) {

    return Filter::Start(__exception_ptr);
}

tResult SimulatorImageFilter::Stop(__exception) {

    Filter::Stop(__exception_ptr);

    RETURN_NOERROR;
}

tResult SimulatorImageFilter::OnCreatePins() {

    m_oColorOutputPin.init(this, adtf_types::GrayPixel::name, PinType::Output);

    RETURN_NOERROR;
}

#define RETURN_IF_FAILED2(s)                                                                                           \
    {                                                                                                                  \
        tResult __result = (s);                                                                                        \
        if (IS_FAILED(__result)) {                                                                                     \
            rx.unlock(image);                                                                                          \
            RETURN_ERROR(__result);                                                                                    \
        }                                                                                                              \
    }
tResult SimulatorImageFilter::ReadAndTransmitData() {

    ImageObject *image = rx.lock(SimulatorSHM::READ_OLDEST);

    if (image == nullptr) {
        failCounter++;
        if (failCounter > 1000) {
            failCounter = 0;
            LOG_ERR << "Simulator: No image from Server";
        }
        usleep(1000);
        RETURN_NOERROR;
    }

    LOG_DBG << "Simulator: Image from Server";

    tTimeStamp tmTimeStamp = 0;
    if (NULL != _clock) {
        tmTimeStamp = _clock->GetStreamTime();
    }

    int imageSz = image->imageWidth * image->imageHeight;

    cObjectPtr<IMediaSample> pColorSample;
    RETURN_IF_FAILED2(AllocMediaSample((tVoid **) &pColorSample));
    RETURN_IF_FAILED2(pColorSample->AllocBuffer(imageSz + static_cast<tInt>(sizeof(smartekCamParam))));

    pColorSample->SetTime(tmTimeStamp);
    pColorSample->SetFlags(m_bSyncEnable ? IMediaSample::MSF_SyncPoint : IMediaSample::MSF_None);

    GrayPixelImage *l_pColorDestBuffer;

    if (IS_OK(pColorSample->WriteLock(reinterpret_cast<void **>(&l_pColorDestBuffer)))) {
        l_pColorDestBuffer->camParams = smartekCamParam;
        std::memcpy(l_pColorDestBuffer->imageData, image->rgbdata, imageSz);
        pColorSample->Unlock(l_pColorDestBuffer);
    }

    rx.unlock(image);

    m_oColorOutputPin.forwardData(pColorSample);

    m_pStatistics->Update();

    RETURN_NOERROR;
}
