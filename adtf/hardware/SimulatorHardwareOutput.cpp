#include "SimulatorHardwareOutput.h"

ADTF_FILTER_PLUGIN("Simulator Hardware Ouput", OID_ADTF_SIMULATOR_HARDWARE_OUTPUT, SimulatorHardwareOutput)

SimulatorHardwareOutput::SimulatorHardwareOutput(const tChar *__info) :
    Filter<cFilter>(__info),
    tx(SimulatorSHM::SERVER, SHM_HWOUT_ID) {
}

SimulatorHardwareOutput::~SimulatorHardwareOutput() {
}

tResult SimulatorHardwareOutput::Init(tInitStage eStage, __exception) {
    RETURN_IF_FAILED(Filter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst) {

    } else if (eStage == StageGraphReady) {
        txBad = !tx.attach();
        failCounter = 0;
    }

    RETURN_NOERROR;
}

tResult SimulatorHardwareOutput::Start(IException **__exception_ptr) {

    RETURN_IF_FAILED(cFilter::Start(__exception_ptr));
    RETURN_NOERROR;
}

tResult SimulatorHardwareOutput::Stop(IException **__exception_ptr) {

    return cFilter::Stop(__exception_ptr);
}

tResult SimulatorHardwareOutput::Shutdown(tInitStage eStage, __exception) {
    RETURN_IF_FAILED(Filter::Shutdown(eStage, __exception_ptr));

    if (eStage == StageFirst) {

    } else if (eStage == StageGraphReady) {
        tx.detach();
    }

    RETURN_NOERROR;
}

tResult SimulatorHardwareOutput::OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2,
                                            IMediaSample *pMediaSample) {

    FILTER_TIMING_START;

    if (pSource == pinCtrlVarsIn && nEventCode == IPinEventSink::PE_MediaSampleReceived) {

        pinCtrlVarsIn.receiveDataCopy_(pMediaSample);

        if (txBad) {
            std::cerr << "Cannot send hardware output: Shared memory could not be opened!" << std::endl;
            RETURN_NOERROR;
        }

        ctrl::ControlVarsVESC ctrlVars = pinCtrlVarsIn.data;
        HardwareOut *hwOut = tx.lock(SimulatorSHM::WRITE_OVERWRITE_OLDEST);

        if (hwOut == nullptr) {
            std::cerr << "Cannot send hardware output: Fifo full!" << std::endl;
            RETURN_NOERROR;
        }

        hwOut->vel = ctrlVars.vel;
        hwOut->deltaFront = ctrlVars.deltaFront;
        hwOut->deltaRear = ctrlVars.deltaRear;

        tx.unlock(hwOut);
    }

    FILTER_TIMING_END;
    RETURN_NOERROR;
}
