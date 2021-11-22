#ifndef SIMULATOR_SPATZ_INPUT_H
#define SIMULATOR_SPATZ_INPUT_H

#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>

#include <chrono>
#include <iostream>
#include <thread>

#include "ADTF_Helper/adtf/Filter.h"
#include "Controller/adtf/adtf_type.h"
#include "Environment/adtf/adtf_type.h"
#include "HardwareIO_SpatzX/adtf/adtf_type_RC.hpp"
#include "SimulatorFilters/lib/shm_ids.h"
#include "SimulatorFilters/lib/shmcomm.h"
#include "Spatz/adtf/adtf_type.h"
#include "Spatz/lib/Spatz.hpp"
#include "Spatzenhirn/adtf/adtf_types.h"

#define OID_ADTF_SIMULATOR_SPATZ_INPUT "adtf.SimulatorSpatzInput"

class SimulatorSpatzInput : public adtf_helper::Filter<adtf::cFilter>, private cKernelThread {

    ADTF_DECLARE_FILTER(OID_ADTF_SIMULATOR_SPATZ_INPUT, "Simulator Spatz Input", OBJCAT_DataFilter)

  public:
    SimulatorSpatzInput(const tChar *__info);
    virtual ~SimulatorSpatzInput();

    tResult Init(tInitStage eStage, __exception = NULL) override;
    tResult Start(__exception = NULL) override;
    tResult Stop(__exception = NULL) override;
    tResult Shutdown(tInitStage eStage, __exception = NULL) override;

  protected:
    tResult ThreadFunc() override;

    SimulatorSHM::SHMComm<HardwareIn> rx;

    int failCounter;

    bool prevPaused;

    Pin<adtf_types::Spatz> pinSpatzOut = {this, "spatz", PinType::Output};
    Pin<adtf_types::RC> pinRCModeOut = {this, "rcmode", PinType::Output};
};

#endif
