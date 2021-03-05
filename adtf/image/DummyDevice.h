//
// Created by gilbertorossi on 03.09.18.
//

#ifndef INC_2020_DUMMYDEVICE_H
#define INC_2020_DUMMYDEVICE_H

#include <ADTF_Helper/adtf/Filter.h>
#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>

#include <sstream>

class DummyDevice : public cBaseIODevice {
  public:
    DummyDevice();
    virtual ~DummyDevice();

    virtual tResult Open(const tChar *strDeviceName, tInt nMode, IException **__exception_ptr);

    virtual tResult Close(IException **__exception_ptr);

    virtual tInt Read(tVoid *pvBuffer, tInt nBufferSize);

    virtual tInt Write(const tVoid *pvBuffer, tInt nBufferSize);

    virtual tInt IOCtl(tInt nCommand, void *pvData, tInt nDataSize);
};


#endif // INC_2020_DUMMYDEVICE_H
