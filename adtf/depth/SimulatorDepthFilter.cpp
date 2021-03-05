#include "SimulatorDepthFilter.h"

#include <SimulatorFilters/lib/SpatzMultiplexer.h>

ADTF_FILTER_PLUGIN("Simulator Depth Input", OID_ADTF_SIMU_CAPTURE_FILTER, SimulatorDepthFilter)

constexpr auto CAMERA_TRANS_X = 0.05F;
constexpr auto CAMERA_TRANS_Y = 0.0F;
constexpr auto CAMERA_TRANS_Z = 0.19F;

SimulatorDepthFilter::SimulatorDepthFilter(const tChar *__info) :
    Filter<cBaseIODeviceFilter>(__info),
    depthRx(SimulatorSHM::CLIENT, SHM_DEPTH_ID),
    spatzRx(SimulatorSHM::CLIENT, SHM_HWIN_ID) {

    m_pDevice = new DummyDevice();
}

SimulatorDepthFilter::~SimulatorDepthFilter() {
}

tResult SimulatorDepthFilter::OpenDevice(__exception) {
    RETURN_NOERROR;
}

auto SimulatorDepthFilter::Init(tInitStage eStage, __exception) -> tResult {

    RETURN_IF_FAILED(Filter::Init(eStage, __exception_ptr));

    if (eStage == StageGraphReady) {
        depthRx.attach();
        spatzRx.attach();
        failCounter = 0;
    }

    RETURN_NOERROR;
}

auto SimulatorDepthFilter::Shutdown(tInitStage eStage, __exception) -> tResult {

    RETURN_IF_FAILED(Filter::Shutdown(eStage, __exception_ptr));

    if (eStage == StageFirst) {

    } else if (eStage == StageGraphReady) {
        depthRx.detach();
        spatzRx.detach();
    }

    RETURN_NOERROR;
}

auto SimulatorDepthFilter::Start(__exception) -> tResult {

    return Filter::Start(__exception_ptr);
}

auto SimulatorDepthFilter::Stop(__exception) -> tResult {

    Filter::Stop(__exception_ptr);

    RETURN_NOERROR;
}

auto SimulatorDepthFilter::OnCreatePins() -> tResult {

    RETURN_NOERROR;
}

#define RETURN_IF_FAILED2(s)                                                                                           \
    {                                                                                                                  \
        tResult __result = (s);                                                                                        \
        if (IS_FAILED(__result)) {                                                                                     \
            rx.unlock(depthImage);                                                                                     \
            RETURN_ERROR(__result);                                                                                    \
        }                                                                                                              \
    }

auto SimulatorDepthFilter::ReadAndTransmitData() -> tResult {

    DepthObject *depthImage = depthRx.lock(SimulatorSHM::READ_OLDEST);

    if (depthImage == nullptr) {
        failCounter++;
        if (failCounter > 1000) {
            failCounter = 0;
            LOG_ERR << "Simulator: No depth image from Server";
        }
        usleep(1000);
        RETURN_NOERROR;
    }

    LOG_DBG << "Simulator: Depth image from Server";

    tTimeStamp tmTimeStamp = 0;
    if (nullptr != _clock) {
        tmTimeStamp = _clock->GetStreamTime();
    }

    /*
     * Get spatz for point cloud
     */
    auto spatz = SpatzMultiplexer::getLastSpatz();

    /*
     * Send actual point cloud
     */
    const auto *const cvDepthPoints = reinterpret_cast<cv::Point3f *>(depthImage->depthPoints);

    SpatzPointcloud spatzPointcloud;
    spatzPointcloud.spatz = spatz;
    spatzPointcloud.width = DEPTH_WIDTH;
    spatzPointcloud.height = DEPTH_HEIGHT;
    spatzPointcloud.pointcloud.resize(spatzPointcloud.width * spatzPointcloud.height);

    SpatzPointcloud resizedSpatzPointcloud;
    resizedSpatzPointcloud.spatz = spatz;
    resizedSpatzPointcloud.width = spatzPointcloud.width / 2;
    resizedSpatzPointcloud.height = spatzPointcloud.height / 2;
    resizedSpatzPointcloud.pointcloud.resize(resizedSpatzPointcloud.width * resizedSpatzPointcloud.height);

    for (std::size_t y = 0; y < spatzPointcloud.height; ++y) {
        for (std::size_t x = 0; x < spatzPointcloud.width; ++x) {
            const auto &d435Pt = cvDepthPoints[y * spatzPointcloud.width + x];
            const cv::Point3f pt{d435Pt.z + CAMERA_TRANS_X, -d435Pt.x + CAMERA_TRANS_Y, -d435Pt.y + CAMERA_TRANS_Z};

            spatzPointcloud.pointcloud[y * spatzPointcloud.width + x] = pt;
            if (x % 2 == 0 and y % 2 == 0) {
                resizedSpatzPointcloud.pointcloud[y / 2 * resizedSpatzPointcloud.width + x / 2] = pt;
            }
        }
    }

    depthRx.unlock(depthImage);

    pointcloudOut.sendData(tmTimeStamp, spatzPointcloud);
    filteredPointcloudOut.sendData(tmTimeStamp, resizedSpatzPointcloud);

    m_pStatistics->Update();

    RETURN_NOERROR;
}
