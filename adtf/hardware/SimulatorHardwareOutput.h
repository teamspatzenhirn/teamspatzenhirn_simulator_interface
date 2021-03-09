#ifndef SIMULATOR_HARDWARE_OUTPUT_H
#define SIMULATOR_HARDWARE_OUTPUT_H

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

#define OID_ADTF_SIMULATOR_HARDWARE_OUTPUT "adtf.SimulatorHardwareOutput"

class SimulatorHardwareOutput : public adtf_helper::Filter<adtf::cFilter> {

    ADTF_DECLARE_FILTER(OID_ADTF_SIMULATOR_HARDWARE_OUTPUT, "Simulator Hardware Output", OBJCAT_DataFilter)

  public:
    SimulatorHardwareOutput(const tChar *__info);
    virtual ~SimulatorHardwareOutput();

    virtual tResult OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2,
                               IMediaSample *pMediaSample) override;

    tResult Init(tInitStage eStage, __exception = NULL) override;
    tResult Start(__exception = NULL) override;
    tResult Stop(__exception = NULL) override;
    tResult Shutdown(tInitStage eStage, __exception = NULL) override;

  protected:
    SimulatorSHM::SHMComm<HardwareOut> tx;
    bool txBad;

    int failCounter;

    Pin<adtf_types::ControlVarsVESC> pinCtrlVarsIn = {this, "ctrlVarsVesc", PinType::Input};
};

#endif
