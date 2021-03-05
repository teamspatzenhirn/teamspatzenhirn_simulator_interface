#include "SimulatorVisualizationOutput.h"

ADTF_FILTER_PLUGIN("Simulator Visualization Output", OID_ADTF_SIMULATOR_VISUALIZATION_OUTPUT,
                   SimulatorVisualizationOutput)

SimulatorVisualizationOutput::SimulatorVisualizationOutput(const tChar *__info) :
    Filter<cFilter>(__info),
    tx(SimulatorSHM::SERVER, SHM_VISOUT_ID) {
}

SimulatorVisualizationOutput::~SimulatorVisualizationOutput() {
}

tResult SimulatorVisualizationOutput::Init(tInitStage eStage, __exception) {
    RETURN_IF_FAILED(Filter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst) {

    } else if (eStage == StageGraphReady) {
        txBad = !tx.attach();
        failCounter = 0;

        if (txBad) {
            std::cerr << "Opening shared memory failed for visualization output failed!";
        }
    }

    RETURN_NOERROR;
}

tResult SimulatorVisualizationOutput::Start(IException **__exception_ptr) {

    RETURN_IF_FAILED(cFilter::Start(__exception_ptr));
    RETURN_NOERROR;
}

tResult SimulatorVisualizationOutput::Stop(IException **__exception_ptr) {

    return cFilter::Stop(__exception_ptr);
}

tResult SimulatorVisualizationOutput::Shutdown(tInitStage eStage, __exception) {
    RETURN_IF_FAILED(Filter::Shutdown(eStage, __exception_ptr));

    if (eStage == StageFirst) {

    } else if (eStage == StageGraphReady) {
        tx.detach();
    }

    RETURN_NOERROR;
}

tResult SimulatorVisualizationOutput::OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2,
                                                 IMediaSample *pMediaSample) {

    FILTER_TIMING_START;

    if (pSource == pinTrajectoryIn && nEventCode == IPinEventSink::PE_MediaSampleReceived) {

        pinTrajectoryIn.receiveDataCopy_(pMediaSample);

        if (txBad) {
            std::cerr << "Cannot send visualization output: Shared memory could not be opened!" << std::endl;
            RETURN_NOERROR;
        }

        VisualizationOut *visOut = tx.lock(SimulatorSHM::WRITE_OVERWRITE_OLDEST);

        if (nullptr == visOut) {
            std::cerr << "Cannot send visualization output: Fifo full!" << std::endl;
        } else {
            std::shared_ptr<traj::AbstractTrajectory> trajectory = pinTrajectoryIn.data;

            if (nullptr != trajectory.get()) {
                double stepSize = (trajectory->endTime() - trajectory->startTime()) / 128.0f;

                for (int i = 0; i < 128; i++) {
                    double t = trajectory->startTime() + stepSize * i;
                    visOut->trajectoryPoints[i] = trajectory->pos(t);
                }
            }

            tx.unlock(visOut);
        }
    }

    FILTER_TIMING_END;
    RETURN_NOERROR;
}
