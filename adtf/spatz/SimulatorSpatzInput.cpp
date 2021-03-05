#include "SimulatorSpatzInput.h"

#include <SimulatorFilters/lib/SpatzMultiplexer.h>

ADTF_FILTER_PLUGIN("Simulator Spatz Input", OID_ADTF_SIMULATOR_SPATZ_INPUT, SimulatorSpatzInput)

SimulatorSpatzInput::SimulatorSpatzInput(const tChar *__info) :
    Filter<cFilter>(__info),
    rx(SimulatorSHM::CLIENT, SHM_HWIN_ID),
    prevBinaryLightSensorTriggered(false),
    prevPaused(true) {
}

SimulatorSpatzInput::~SimulatorSpatzInput() {
}

tResult SimulatorSpatzInput::Init(tInitStage eStage, __exception) {
    RETURN_IF_FAILED(Filter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst) {

    } else if (eStage == StageGraphReady) {
        rx.attach();
        failCounter = 0;
        RETURN_IF_FAILED(cKernelThread::Create(cKernelThread::TF_Suspended));
    }

    RETURN_NOERROR;
}

tResult SimulatorSpatzInput::Start(IException **__exception_ptr) {

    RETURN_IF_FAILED(cFilter::Start(__exception_ptr));
    RETURN_IF_FAILED(cKernelThread::Run(tTrue));
    RETURN_NOERROR;
}

tResult SimulatorSpatzInput::Stop(IException **__exception_ptr) {

    cKernelThread::Suspend(tTrue);
    return cFilter::Stop(__exception_ptr);
}

tResult SimulatorSpatzInput::Shutdown(tInitStage eStage, __exception) {
    RETURN_IF_FAILED(Filter::Shutdown(eStage, __exception_ptr));

    if (eStage == StageFirst) {

    } else if (eStage == StageGraphReady) {
        cKernelThread::Release();
        rx.detach();
    }

    RETURN_NOERROR;
}

tResult SimulatorSpatzInput::ThreadFunc() {

    HardwareIn *inobj = rx.lock(SimulatorSHM::READ_NEWEST);

    if (inobj == nullptr) {
        usleep(1000);
        failCounter++;
        if (failCounter > 1000) {
            failCounter = 0;
            LOG_ERR << "Cannot get hardware input: Server not sending messages";
        }
    } else {
        failCounter = 0;

        // Sending spatz:

        env::Spatz spatz(0, cv::Point3d{inobj->x, inobj->y, inobj->psi});
        spatz.setVel(cv::Point3d{inobj->velX, inobj->velY, 0});
        spatz.setSteerAngle(inobj->steeringAngle);
        spatz.setdRot(cv::Point3d(0, 0, inobj->dPsi));
        spatz.setAcc(cv::Point3d{inobj->accX, inobj->accY, 0});
        spatz.alpha_front = inobj->alphaFront;
        spatz.alpha_rear = inobj->alphaRear;
        spatz.setLaser(inobj->laserSensorValue);
        spatz.setT(inobj->time);
        spatz.setIntegratedDistance(inobj->drivenDistance);

        bool changed = prevBinaryLightSensorTriggered != inobj->binaryLightSensorTriggered;
        prevBinaryLightSensorTriggered = inobj->binaryLightSensorTriggered;

        spatz.setSensorIrSide(std::make_pair(changed, std::make_pair(false, inobj->binaryLightSensorTriggered)));

        pinSpatzOut.sendData(_clock->GetStreamTime(), spatz);
        SpatzMultiplexer::submitSpatz(spatz);

        // Sending RCMode:

        LOG_DBG << "Paused: " << inobj->paused;

        if (prevPaused != inobj->paused) {
            hw::RCMode mode = inobj->paused ? hw::RCMode::remote : hw::RCMode::adtf;
            pinRCModeOut.sendData(_clock->GetStreamTime(), mode);
        }

        prevPaused = inobj->paused;

        rx.unlock(inobj);
    }

    RETURN_NOERROR;
}
