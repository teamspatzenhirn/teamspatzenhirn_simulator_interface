#ifndef SIMULATOR_VISUALIZATION_OUTPUT_H
#define SIMULATOR_VISUALIZATION_OUTPUT_H

#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>

#include <chrono>
#include <iostream>
#include <thread>

#include "ADTF_Helper/adtf/Filter.h"
#include "Controller/adtf/adtf_type.h"
#include "Environment/adtf/adtf_type.h"
#include "SimulatorFilters/lib/shm_ids.h"
#include "SimulatorFilters/lib/shmcomm.h"
#include "Spatz/adtf/adtf_type.h"
#include "Spatz/lib/Spatz.hpp"
#include "Spatzenhirn/adtf/adtf_types.h"
#include "Trajectory/adtf/adtf_types.h"
#include "Trajectory/lib/AbstractTrajectory.h"

#define OID_ADTF_SIMULATOR_VISUALIZATION_OUTPUT "adtf.SimulatorVisualizationOutput"

class SimulatorVisualizationOutput : public adtf_helper::Filter<adtf::cFilter> {

    ADTF_DECLARE_FILTER(OID_ADTF_SIMULATOR_VISUALIZATION_OUTPUT, "Simulator Visualization Output", OBJCAT_DataFilter)

  public:
    SimulatorVisualizationOutput(const tChar *__info);
    virtual ~SimulatorVisualizationOutput();

    virtual tResult OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2,
                               IMediaSample *pMediaSample) override;

    tResult Init(tInitStage eStage, __exception = NULL) override;
    tResult Start(__exception = NULL) override;
    tResult Stop(__exception = NULL) override;
    tResult Shutdown(tInitStage eStage, __exception = NULL) override;

  protected:
    SimulatorSHM::SHMComm<VisualizationOut> tx;
    bool txBad;

    int failCounter;

    Pin<adtf_types::Trajectory> pinTrajectoryIn = {this, "trajectory", PinType::Input};
};

#endif
