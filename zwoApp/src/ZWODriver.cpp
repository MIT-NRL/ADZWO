#include "ZWODriver.h"
#include "ASICamera2.h"
#include "asynDriver.h"
#include "epicsExport.h"

#include <stdio.h>
#include <string.h>
#include <string>

#include <epicsEvent.h>
#include <epicsMutex.h>
#include <epicsThread.h>
#include <epicsTime.h>
#include <iocsh.h>

static const char *driverName = "ZWODriver";
static const char *driverVersion = "0.0.1";

ZWODriver::ZWODriver(const char *portName, int maxBuffers, size_t maxMemory,
                     int priority, int stackSize)
    : ADDriver(portName, 1, 0, maxBuffers, maxMemory, 0,
               0,    /* No interfaces beyond those set in ADDriver.cpp */
               0, 1, /* ASYN_CANBLOCK=0, ASYN_MULTIDEVICE=0, autoConnect=1 */
               priority, stackSize) {
    int status = asynSuccess;
    const char *functionName = "ZWODriver";

    // Connect to camera
    int cameraID;
    status = this->connectCamera(&cameraID);
    if (status != asynSuccess) {
        return;
    }

    ASI_CAMERA_INFO cameraInfo;
    ASIGetCameraPropertyByID(cameraID, &cameraInfo);

    // Set some initial values for various parameters
    status = setStringParam(ADManufacturer, "ZWO");
    status |= setStringParam(ADModel, cameraInfo.Name);
    status |= setStringParam(ADSerialNumber, "N/A");
    status |= setStringParam(ADFirmwareVersion, "N/A");
    status |= setStringParam(NDDriverVersion, driverVersion);
    status |= setStringParam(ADSDKVersion, ASIGetSDKVersion());

    status |= setIntegerParam(ADSizeX, cameraInfo.MaxWidth);
    status |= setIntegerParam(ADSizeY, cameraInfo.MaxHeight);
    status |= setIntegerParam(ADMaxSizeX, cameraInfo.MaxWidth);
    status |= setIntegerParam(ADMaxSizeY, cameraInfo.MaxHeight);

    return;
}

asynStatus ZWODriver::connectCamera(int *cameraIDOut) {
    ASI_ERROR_CODE asiStatus;

    int numConnectedCameras = ASIGetNumOfConnectedCameras();
    if (numConnectedCameras == 0) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: No ASI camera connected.\n", driverName, __func__);
        return asynError;
    }

    // TODO: Implement support for multiple cameras
    ASI_CAMERA_INFO cameraInfo;
    for (int cameraIndex = 0; cameraIndex < numConnectedCameras;
         cameraIndex++) {
        ASIGetCameraProperty(&cameraInfo, cameraIndex);

        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                  "%s:%s: Found camera \"%s\" with id %d\n", driverName,
                  __func__, cameraInfo.Name, cameraInfo.CameraID);
    }

    int cameraID = cameraInfo.CameraID;

    // Initialize camera
    asiStatus = ASIOpenCamera(cameraID);
    if (asiStatus != ASI_SUCCESS) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: Failed to open camera (id: %d, error: %d)\n",
                  driverName, __func__, cameraID, asiStatus);
        return asynError;
    }

    asiStatus = ASIInitCamera(cameraID);
    if (asiStatus != ASI_SUCCESS) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: Failed to init camera (id: %d, error: %d)\n",
                  driverName, __func__, cameraID, asiStatus);
        return asynError;
    }

    *cameraIDOut = cameraID;
    return asynSuccess;
}

/** Code for iocsh registration */
extern "C" int ZWODriverConfig(const char *portName, int maxBuffers,
                               size_t maxMemory, int priority, int stackSize) {
    new ZWODriver(portName, maxBuffers, maxMemory, priority, stackSize);
    return (asynSuccess);
}

/** Code for iocsh registration */
static const iocshArg ZWODriverConfigArg0 = {"Port name", iocshArgString};
static const iocshArg ZWODriverConfigArg1 = {"maxBuffers", iocshArgInt};
static const iocshArg ZWODriverConfigArg2 = {"maxMemory", iocshArgInt};
static const iocshArg ZWODriverConfigArg3 = {"priority", iocshArgInt};
static const iocshArg ZWODriverConfigArg4 = {"stackSize", iocshArgInt};
static const iocshArg *const ZWODriverConfigArgs[] = {
    &ZWODriverConfigArg0, &ZWODriverConfigArg1, &ZWODriverConfigArg2,
    &ZWODriverConfigArg3, &ZWODriverConfigArg4,
};
static const iocshFuncDef configURLDriver = {"ZWODriverConfig", 5,
                                             ZWODriverConfigArgs};
static void configURLDriverCallFunc(const iocshArgBuf *args) {
    ZWODriverConfig(args[0].sval, args[1].ival, args[2].ival, args[3].ival,
                    args[4].ival);
}

static void ZWODriverRegister(void) {
    iocshRegister(&configURLDriver, configURLDriverCallFunc);
}

extern "C" {
epicsExportRegistrar(ZWODriverRegister);
}
