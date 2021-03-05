//
// Created by gilbertorossi on 03.09.18.
//

#include "DummyDevice.h"

tResult DummyDevice::Open(const tChar *strDeviceName, tInt nMode, IException **__exception_ptr) {
    RETURN_NOERROR;
}

tResult DummyDevice::Close(IException **__exception_ptr) {
    RETURN_NOERROR;
}

tInt DummyDevice::Read(tVoid *pvBuffer, tInt nBufferSize) {
    RETURN_NOERROR;
}

tInt DummyDevice::Write(const tVoid *pvBuffer, tInt nBufferSize) {
    RETURN_NOERROR;
}

tInt DummyDevice::IOCtl(tInt nCommand, void *pvData, tInt nDataSize) {
    RETURN_NOERROR;
}
DummyDevice::DummyDevice() {
}
DummyDevice::~DummyDevice() {
}
