#include "ZWODriver.h"
#include "ADDriver.h"
#include "ASICamera2.h"
#include "NDArray.h"
#include "NDAttribute.h"
#include "asynDriver.h"
#include "epicsExport.h"
#include "epicsStdio.h"
#include "epicsTypes.h"

#include <cstdint>
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

static void ZWODriverCaptureTaskC(void *drvPvt) {
    ZWODriver *driver = (ZWODriver *)drvPvt;
    driver->captureTask();
}

static void ZWODriverPollingTaskC(void *drvPvt) {
    ZWODriver *driver = (ZWODriver *)drvPvt;
    driver->pollingTask();
}

ZWODriver::ZWODriver(const char *portName, int maxBuffers, size_t maxMemory,
                     int priority, int stackSize)
    : ADDriver(portName, 1, 0, maxBuffers, maxMemory, 0,
               0,    /* No interfaces beyond those set in ADDriver.cpp */
               0, 1, /* ASYN_CANBLOCK=0, ASYN_MULTIDEVICE=0, autoConnect=1 */
               priority, stackSize) {
    printf("\n\n\n\n\n");

    int status = asynSuccess;

    this->startEvent = new epicsEvent();
    this->stopEvent = new epicsEvent();

    this->cameraID = -1;
    this->connect(this->pasynUserSelf);

    // // THIS IS JUST FOR TESTING
    // int numControls;
    // ASIGetNumOfControls(cameraID, &numControls);
    // printf("NumControls: %d\n", numControls);

    // for (int i = 0; i < numControls; i++) {
    //     ASI_CONTROL_CAPS caps;
    //     ASIGetControlCaps(cameraID, i, &caps);

    //     printf("Caps %s\n", caps.Description);
    // }
    // // Setup Camera
    ASISetROIFormat(cameraID, cameraInfo.MaxWidth, cameraInfo.MaxHeight, 1,
                    ASI_IMG_RAW16);
    setIntegerParam(NDColorMode, NDColorModeMono);
    setIntegerParam(NDDataType, NDUInt16);

    // Create the thread that performs the image capturing
    status = (epicsThreadCreate("ZWODriverCaptureTask", epicsThreadPriorityHigh,
                                epicsThreadGetStackSize(epicsThreadStackMedium),
                                (EPICSTHREADFUNC)ZWODriverCaptureTaskC,
                                this) == NULL);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s epicsThreadCreate failure for ZWODriverCaptureTask.\n",
                  driverName, __func__);
        return;
    }

    // Create the thread that periodically reads the temperature, etc.
    status = (epicsThreadCreate(
                  "ZWODriverPollingTask", epicsThreadPriorityMedium,
                  epicsThreadGetStackSize(epicsThreadStackMedium),
                  (EPICSTHREADFUNC)ZWODriverPollingTaskC, this) == NULL);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s epicsThreadCreate failure for ZWODriverPollingTask.\n",
                  driverName, __func__);
        return;
    }

    printf("\n\n\n\n\n");
    return;
}

ZWODriver::~ZWODriver() { disconnect(this->pasynUserSelf); }

asynStatus ZWODriver::connect(asynUser *pasynUser) {
    disconnectCamera();
    return connectCamera();
}

asynStatus ZWODriver::disconnect(asynUser *pasynUser) {
    return this->disconnectCamera();
}

asynStatus ZWODriver::writeInt32(asynUser *pasynUser, epicsInt32 value) {
    int function = pasynUser->reason;
    int status = asynSuccess;

    if (function == ADAcquire) {
        if (value == 1) {
            startEvent->signal();
        }

        if (value == 0) {
            stopEvent->signal();
        }
    }

    status |= ADDriver::writeInt32(pasynUser, value);

    return (asynStatus)status;
}

asynStatus ZWODriver::writeFloat64(asynUser *pasynUser, epicsFloat64 value) {
    int function = pasynUser->reason;
    int status = asynSuccess;

    if (function == ADAcquireTime) {
        long exposureTime = value * 1000 * 1000;
        status |= (ASISetControlValue(cameraID, ASI_EXPOSURE, exposureTime,
                                      ASI_FALSE) == ASI_SUCCESS
                       ? asynSuccess
                       : asynError);
    } else if (function == ADGain) {
        status |= (ASISetControlValue(cameraID, ASI_GAIN, (long)value,
                                      ASI_FALSE) == ASI_SUCCESS
                       ? asynSuccess
                       : asynError);
    } else if (function == ADTemperature) {
        int targetTemp = value;
        status |= (ASISetControlValue(cameraID, ASI_TARGET_TEMP, targetTemp,
                                      ASI_FALSE) == ASI_SUCCESS
                       ? asynSuccess
                       : asynError);
    }

    status |= ADDriver::writeFloat64(pasynUser, value);
    return (asynStatus)status;
}

asynStatus ZWODriver::connectCamera() {
    int status;
    ASI_ERROR_CODE asiStatus;

    int numConnectedCameras = ASIGetNumOfConnectedCameras();
    if (numConnectedCameras == 0) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: No ASI camera connected.\n", driverName, __func__);
        return asynError;
    }

    // TODO: Implement support for multiple cameras
    //       This can be done using ASIGetID
    ASI_CAMERA_INFO cameraInfo;
    for (int cameraIndex = 0; cameraIndex < numConnectedCameras;
         cameraIndex++) {
        ASIGetCameraProperty(&cameraInfo, cameraIndex);
        epicsStdoutPrintf("%s:%s: Found camera \"%s\" with id %d\n", driverName,
                          __func__, cameraInfo.Name, cameraInfo.CameraID);
    }

    int cameraID = cameraInfo.CameraID;

    // Initialize camera
    asiStatus = ASIOpenCamera(cameraID);
    if (asiStatus) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: Failed to open camera (id: %d, error: %d) - "
                  "Try running as root\n",
                  driverName, __func__, cameraID, asiStatus);

        return asynError;
    }

    asiStatus = ASIInitCamera(cameraID);
    if (asiStatus) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: Failed to init camera (id: %d, error: %d)\n",
                  driverName, __func__, cameraID, asiStatus);
        return asynError;
    }

    //
    ASIGetCameraPropertyByID(cameraID, &cameraInfo);
    this->cameraID = cameraID;
    this->cameraInfo = cameraInfo;

    // Print control capabilities
    int numControls;
    ASIGetNumOfControls(cameraID, &numControls);

    for (int i = 0; i < numControls; i++) {
        ASI_CONTROL_CAPS caps;
        ASIGetControlCaps(cameraID, i, &caps);

        epicsStdoutPrintf("%s:%s: Control %s, %ld-%ld, default %ld\n",
                          driverName, __func__, caps.Name, caps.MinValue,
                          caps.MaxValue, caps.DefaultValue);
    }

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

    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: unable to set camera parameters on camera %d\n",
                  driverName, __func__, cameraID);
        return asynError;
    }

    // Configure cooling
    if (cameraInfo.IsCoolerCam) {
        ASISetControlValue(cameraID, ASI_COOLER_ON, 1, ASI_FALSE);

        epicsFloat64 targetTemp;
        status |= getDoubleParam(ADTemperature, &targetTemp);
        ASISetControlValue(cameraID, ASI_TARGET_TEMP, (int)targetTemp,
                           ASI_FALSE);
    }

    // Print ID
    ASI_ID asiId;
    ASIGetID(cameraInfo.CameraID, &asiId);

    char idString[32] = {0};
    sprintf(idString, "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X", asiId.id[0],
            asiId.id[1], asiId.id[2], asiId.id[3], asiId.id[4], asiId.id[5],
            asiId.id[6], asiId.id[7]);

    epicsStdoutPrintf("%s:%s: Did connect to camera \"%s\" with id %d (%s)\n",
                      driverName, __func__, cameraInfo.Name,
                      cameraInfo.CameraID, idString);

    return asynSuccess;
}

asynStatus ZWODriver::disconnectCamera() {
    if (cameraID < 0)
        return asynDisconnected;

    int status = ASI_SUCCESS;
    status |= ASIStopExposure(cameraID);
    status |= ASICloseCamera(cameraID);

    cameraID = -1;
    return (status == ASI_SUCCESS) ? asynSuccess : asynError;
}

void ZWODriver::captureTask() {
    int status = asynSuccess;
    int minX, minY, sizeX, sizeY, maxSizeX, maxSizeY;
    int bin = 1;

    int imageCounter;
    int numImages, numImagesCounter;
    epicsTimeStamp startTime, endTime;

    getIntegerParam(ADMinX, &minX);
    getIntegerParam(ADMinY, &minY);
    getIntegerParam(ADSizeX, &sizeX);
    getIntegerParam(ADSizeY, &sizeY);
    // getIntegerParam(ADBinX, &camBinX);
    // getIntegerParam(ADBinY, &camBinY);

    status |= getIntegerParam(ADMinX, &minX);
    status |= getIntegerParam(ADMinY, &minY);
    status |= getIntegerParam(ADSizeX, &sizeX);
    status |= getIntegerParam(ADSizeY, &sizeY);
    status |= getIntegerParam(ADMaxSizeX, &maxSizeX);
    status |= getIntegerParam(ADMaxSizeY, &maxSizeY);

    // if (minX + sizeX > maxSizeX) {
    //     sizeX = maxSizeX - minX;
    //     setIntegerParam(ADSizeX, sizeX);
    // }

    // if (minY + sizeY > maxSizeY) {
    //     sizeY = maxSizeY - minY;
    //     setIntegerParam(ADSizeY, sizeY);
    // }

    // Set all the parameters

    ASI_EXPOSURE_STATUS exposureStatus;

    while (true) {
        if (cameraID < 0) {
            epicsThreadSleep(1);
            continue;
        }

        bool signal = this->startEvent->wait(1);
        if (!signal)
            continue;

        this->lock();

        // TODO: Send parameters to camera

        this->unlock();

        // Wait until camera is ready to start with exposure
        while (!ASIGetExpStatus(cameraID, &exposureStatus) &&
               exposureStatus == ASI_EXP_WORKING) {
            epicsThreadSleep(SHORT_WAIT);
        }

        if (ASIStartExposure(cameraID, ASI_FALSE)) {
            // FAILED
            printf("------------------------ Failed to start exposure\n");
            continue;
        }

        setIntegerParam(ADStatus, ADStatusAcquire);
        callParamCallbacks();
        epicsTimeGetCurrent(&startTime);

        // Wait until image has been acquired
        while (!ASIGetExpStatus(cameraID, &exposureStatus) &&
               exposureStatus == ASI_EXP_WORKING) {
            epicsThreadSleep(SHORT_WAIT);
        }

        if (exposureStatus == ASI_EXP_SUCCESS) {
            printf("------------------------ EXP SUCCESS\n");

            // Update counters
            getIntegerParam(NDArrayCounter, &imageCounter);
            getIntegerParam(ADNumImagesCounter, &numImagesCounter);
            numImagesCounter++;
            imageCounter++;
            setIntegerParam(NDArrayCounter, imageCounter);
            setIntegerParam(ADNumImagesCounter, numImagesCounter);

            // XXXXXXXXXXXXXXXXXXXX
            size_t dims[2] = {(size_t)maxSizeX, (size_t)maxSizeY};
            NDArray *pImage =
                this->pNDArrayPool->alloc(2, dims, NDUInt16, 0, NULL);

            pImage->uniqueId = imageCounter;
            pImage->timeStamp = startTime.secPastEpoch + startTime.nsec / 1.e9;
            updateTimeStamp(&pImage->epicsTS);

            ASIGetDataAfterExp(cameraID, (unsigned char *)pImage->pData,
                               pImage->dataSize);

            for (int i = 0; i < 128; i++) {
                printf("%d\n", ((uint16_t *)pImage->pData)[i]);
            }

            doCallbacksGenericPointer(pImage, NDArrayData, 0);
            pImage->release();
        } else {
            // ERROR
            printf("------------------------ EXP FAILED\n");
        }

        // setIntegerParam(ADStatus, ADStatusIdle);

        callParamCallbacks();
        // Complete Acquire callback
        setIntegerParam(ADAcquire, 0);
        callParamCallbacks();
    }
}

void ZWODriver::pollingTask() {
    epicsFloat64 timeout = 1;

    long cValue;
    ASI_BOOL cAuto;

    while (true) {
        epicsThreadSleep(timeout);
        if (cameraID < 0)
            continue;

        lock();

        if (ASIGetControlValue(cameraID, ASI_TEMPERATURE, &cValue, &cAuto) ==
            ASI_SUCCESS) {
            double temperature = (double)(cValue) / 10.0;
            setDoubleParam(ADTemperatureActual, temperature);
        }

        callParamCallbacks();
        unlock();
    }
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
