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
static const char *driverVersion = "0.3.0";

static void ZWODriverCaptureTaskC(void *drvPvt) {
    ZWODriver *driver = (ZWODriver *)drvPvt;
    driver->captureTask();
}

static void ZWODriverPollingTaskC(void *drvPvt) {
    ZWODriver *driver = (ZWODriver *)drvPvt;
    driver->pollingTask();
}

static long clampUnsignedRange(long value, unsigned long minValue,
                               unsigned long maxValue) {
    if ((minValue == 0) && (maxValue == 0)) {
        return value;
    }
    if (value < (long)minValue) {
        return (long)minValue;
    }
    if (value > (long)maxValue) {
        return (long)maxValue;
    }
    return value;
}

static long clampSignedRange(long value, long minValue, long maxValue) {
    if ((minValue == 0) && (maxValue == 0)) {
        return value;
    }
    if (value < minValue) {
        return minValue;
    }
    if (value > maxValue) {
        return maxValue;
    }
    return value;
}

ZWODriver::ZWODriver(const char *portName, int maxBuffers, size_t maxMemory,
                     int priority, int stackSize)
    : ADDriver(portName, 1, 0, maxBuffers, maxMemory, 0,
               0,    /* No interfaces beyond those set in ADDriver.cpp */
               0, 0, /* ASYN_CANBLOCK=0, ASYN_MULTIDEVICE=0, autoConnect=0 */
               priority, stackSize) {
    memset(&this->cameraInfo, 0, sizeof(this->cameraInfo));
    memset(&this->controlLimits, 0, sizeof(this->controlLimits));

    createParam(ADOffsetString, asynParamFloat64, &ADOffset);
    createParam(ADCoolerPowerPercString, asynParamInt32,
                &ADCoolerPowerPerc);
    createParam(ADSensorPixelSizeString, asynParamFloat64,
                &ADSensorPixelSize);
    createParam(ADUSBBandwidthString, asynParamInt32,
                &ADUSBBandwidth);
    createParam(ADUSBBandwidthAutoString, asynParamInt32,
                &ADUSBBandwidthAuto);
    createParam(ADCameraConnectString, asynParamInt32, &ADCameraConnect);

    int status = asynSuccess;

    this->startEvent = new epicsEvent();
    this->stopEvent = new epicsEvent();

    this->cameraID = -1;
    this->deviceIsReachable = true;
    asynPortDriver::connect(this->pasynUserSelf);
    setIntegerParam(ADCameraConnect, 0);
    connectCamera();

    // Set default values
    status |= setIntegerParam(
        NDColorMode, cameraInfo.IsColorCam ? NDColorModeRGB3 : NDColorModeMono);
    status |= setIntegerParam(NDDataType, NDUInt8);

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
    return;
}

ZWODriver::~ZWODriver() { disconnect(this->pasynUserSelf); }

asynStatus ZWODriver::connect(asynUser *pasynUser) {
    (void)pasynUser;
    disconnectCamera("Disconnected");
    return connectCamera();
}

asynStatus ZWODriver::disconnect(asynUser *pasynUser) {
    (void)pasynUser;
    return this->disconnectCamera("Disconnected");
}

void ZWODriver::clearPendingStopEvent() {
    while (this->stopEvent->tryWait()) {
    }
}

asynStatus ZWODriver::setConnectionState(bool connected,
                                         const char *statusMessage) {
    this->deviceIsReachable = true;
    setIntegerParam(ADCameraConnect, connected ? 1 : 0);
    setIntegerParam(ADAcquire, 0);
    setDoubleParam(ADTimeRemaining, 0.0);
    setIntegerParam(ADStatus, connected ? ADStatusIdle : ADStatusDisconnected);
    setStringParam(ADStatusMessage, statusMessage != NULL
                                        ? statusMessage
                                        : (connected ? "Idle" : "Disconnected"));
    callParamCallbacks();
    return asynSuccess;
}

asynStatus ZWODriver::attemptReconnectOnce(const char *reason,
                                           bool *didReconnect) {
    int wasAcquiring = 0;
    asynStatus status;
    if (didReconnect != NULL) {
        *didReconnect = false;
    }

    getIntegerParam(ADAcquire, &wasAcquiring);

    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: %s; attempting single reconnect\n", driverName,
              __func__, reason != NULL ? reason : "Camera disconnected");

    setStringParam(ADStatusMessage, "Attempting reconnect");
    callParamCallbacks();

    if (cameraID >= 0) {
        disconnectCamera("Attempting reconnect");
    }

    status = connectCamera();
    if (status == asynSuccess) {
        if (didReconnect != NULL) {
            *didReconnect = true;
        }
        if (wasAcquiring) {
            setIntegerParam(ADAcquire, 1);
            callParamCallbacks();
        }
    }

    return status;
}

asynStatus ZWODriver::handleCameraError(const char *operation,
                                        ASI_ERROR_CODE asiStatus,
                                        bool *didReconnect) {
    char message[128];
    epicsSnprintf(message, sizeof(message), "Disconnected: %s failed (%d)",
                  operation, asiStatus);

    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: %s\n", driverName, __func__, message);

    if (attemptReconnectOnce(message, didReconnect) == asynSuccess) {
        return asynSuccess;
    }

    return asynDisconnected;
}

asynStatus ZWODriver::checkCameraConnection(const char *operation,
                                            bool *didReconnect) {
    ASI_CAMERA_INFO info;
    ASI_ERROR_CODE asiStatus;
    int numConnectedCameras;
    bool foundCamera = false;

    if (didReconnect != NULL) {
        *didReconnect = false;
    }

    if (cameraID < 0) {
        return asynDisconnected;
    }

    numConnectedCameras = ASIGetNumOfConnectedCameras();
    if (numConnectedCameras <= 0) {
        if (attemptReconnectOnce("Disconnected: camera not present",
                                 didReconnect) == asynSuccess) {
            return asynSuccess;
        }
        return asynDisconnected;
    }

    for (int cameraIndex = 0; cameraIndex < numConnectedCameras; cameraIndex++) {
        asiStatus = ASIGetCameraProperty(&info, cameraIndex);
        if (asiStatus != ASI_SUCCESS) {
            return handleCameraError(operation, asiStatus);
        }
        if (info.CameraID == cameraID) {
            foundCamera = true;
            break;
        }
    }

    if (!foundCamera) {
        if (attemptReconnectOnce("Disconnected: camera ID no longer present",
                                 didReconnect) == asynSuccess) {
            return asynSuccess;
        }
        return asynDisconnected;
    }

    asiStatus = ASIGetCameraPropertyByID(cameraID, &info);
    if (asiStatus != ASI_SUCCESS) {
        return handleCameraError(operation, asiStatus, didReconnect);
    }

    return asynSuccess;
}

asynStatus ZWODriver::applyCachedSettingsToCamera() {
    ASI_ERROR_CODE asiStatus;
    epicsFloat64 doubleValue;
    long controlValue;
    int intValue;
    ASI_BOOL isAuto = ASI_FALSE;

    if (cameraID < 0) {
        return asynDisconnected;
    }

    getDoubleParam(ADAcquireTime, &doubleValue);
    controlValue =
        clampUnsignedRange((long)(doubleValue * 1000.0 * 1000.0),
                           this->controlLimits.minExposure,
                           this->controlLimits.maxExposure);
    asiStatus =
        ASISetControlValue(cameraID, ASI_EXPOSURE, controlValue, ASI_FALSE);
    if (asiStatus != ASI_SUCCESS) {
        return asynError;
    }
    setDoubleParam(ADAcquireTime, (epicsFloat64)controlValue / 1000.0 / 1000.0);

    getDoubleParam(ADGain, &doubleValue);
    controlValue = clampUnsignedRange((long)doubleValue,
                                      this->controlLimits.minGain,
                                      this->controlLimits.maxGain);
    asiStatus = ASISetControlValue(cameraID, ASI_GAIN, controlValue, ASI_FALSE);
    if (asiStatus != ASI_SUCCESS) {
        return asynError;
    }
    setDoubleParam(ADGain, (epicsFloat64)controlValue);

    getDoubleParam(ADOffset, &doubleValue);
    controlValue = clampUnsignedRange((long)doubleValue,
                                      this->controlLimits.minOffset,
                                      this->controlLimits.maxOffset);
    asiStatus =
        ASISetControlValue(cameraID, ASI_OFFSET, controlValue, ASI_FALSE);
    if (asiStatus != ASI_SUCCESS) {
        return asynError;
    }
    setDoubleParam(ADOffset, (epicsFloat64)controlValue);

    if (cameraInfo.IsCoolerCam) {
        asiStatus = ASISetControlValue(cameraID, ASI_COOLER_ON, 1, ASI_FALSE);
        if (asiStatus != ASI_SUCCESS) {
            return asynError;
        }

        getDoubleParam(ADTemperature, &doubleValue);
        controlValue = clampSignedRange((long)doubleValue,
                                        this->controlLimits.minTemp,
                                        this->controlLimits.maxTemp);
        asiStatus = ASISetControlValue(cameraID, ASI_TARGET_TEMP, controlValue,
                                       ASI_FALSE);
        if (asiStatus != ASI_SUCCESS) {
            return asynError;
        }
        setDoubleParam(ADTemperature, (epicsFloat64)controlValue);
    }

    getIntegerParam(ADUSBBandwidthAuto, &intValue);
    isAuto = intValue ? ASI_TRUE : ASI_FALSE;

    getIntegerParam(ADUSBBandwidth, &intValue);
    controlValue = clampUnsignedRange((long)intValue, this->controlLimits.minUSB,
                                      this->controlLimits.maxUSB);
    asiStatus =
        ASISetControlValue(cameraID, ASI_BANDWIDTHOVERLOAD, controlValue, isAuto);
    if (asiStatus != ASI_SUCCESS) {
        return asynError;
    }

    asiStatus = ASIGetControlValue(cameraID, ASI_BANDWIDTHOVERLOAD, &controlValue,
                                   &isAuto);
    if (asiStatus != ASI_SUCCESS) {
        return asynError;
    }
    setIntegerParam(ADUSBBandwidth, (int)controlValue);
    setIntegerParam(ADUSBBandwidthAuto, (int)isAuto);

    return asynSuccess;
}

asynStatus ZWODriver::writeInt32(asynUser *pasynUser, epicsInt32 value) {
    int function = pasynUser->reason;
    int status = asynSuccess;

    int acquiring;
    getIntegerParam(ADAcquire, &acquiring);

    if (function == ADAcquire) {
        if ((value == 1) && (cameraID < 0)) {
            if (attemptReconnectOnce("Acquire requested while disconnected") ==
                asynSuccess) {
                getIntegerParam(ADAcquire, &acquiring);
            } else {
                setIntegerParam(ADAcquire, 0);
                setIntegerParam(ADStatus, ADStatusDisconnected);
                setStringParam(ADStatusMessage, "Camera is disconnected");
                callParamCallbacks();
                return asynDisconnected;
            }
        }

        if (value == 1 && !acquiring) {
            startEvent->signal();
        }

        if (value == 0 && acquiring) {
            stopEvent->signal();
        }
    }

    if (function == ADCameraConnect) {
        if (value) {
            if (cameraID >= 0) {
                status |= setIntegerParam(ADCameraConnect, 1);
                status |= callParamCallbacks();
                return (asynStatus)status;
            }
            return connectCamera();
        }

        if (cameraID >= 0) {
            return disconnectCamera("Disconnected");
        }

        status |= setIntegerParam(ADCameraConnect, 0);
        status |= setIntegerParam(ADStatus, ADStatusDisconnected);
        status |= setStringParam(ADStatusMessage, "Disconnected");
        status |= callParamCallbacks();
        return (asynStatus)status;
    }

    if ((function == ADBinX) || (function == ADBinY)) {
        if (cameraID < 0) {
            if (value < 1) {
                value = 1;
            }
            status |= setIntegerParam(ADBinX, value);
            status |= setIntegerParam(ADBinY, value);
            status |= callParamCallbacks();
            return (asynStatus)status;
        }
        // Keep BinX and BinY in sync, and ensure that they are valid values
        for (int i = 0; i < 16; i++) {
            if (cameraInfo.SupportedBins[i] == 0)
                break;
            if (cameraInfo.SupportedBins[i] == value) {
                status |= setIntegerParam(ADBinX, value);
                status |= setIntegerParam(ADBinY, value);
                status |= callParamCallbacks();
                return (asynStatus)status;
            }
        }

        return asynError;
    }

    if (function == NDDataType) {
        if ((value != NDUInt8) && (value != NDUInt16)) {
            return asynError;
        }
    }

    if (function == ADUSBBandwidth) {
        int autoMode;
        getIntegerParam(ADUSBBandwidthAuto, &autoMode);
        if (autoMode) {
            // Optionally log or set a status message
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s:%s: Cannot set USB bandwidth in auto mode\n",
                      driverName, __func__);
            return asynDisabled; // Ignore or reject in auto mode
        }
        value = (epicsInt32)clampUnsignedRange((long)value,
                                               this->controlLimits.minUSB,
                                               this->controlLimits.maxUSB);
        if (cameraID >= 0) {
            status |= ASISetControlValue(cameraID, ASI_BANDWIDTHOVERLOAD, value,
                                         ASI_FALSE);
            long bandwidthValue;
            ASI_BOOL isAuto = ASI_FALSE;
            ASIGetControlValue(cameraID, ASI_BANDWIDTHOVERLOAD, &bandwidthValue,
                               &isAuto);
            status |= setIntegerParam(ADUSBBandwidth, bandwidthValue);
            status |= setIntegerParam(ADUSBBandwidthAuto, (int)isAuto);
        } else {
            status |= setIntegerParam(ADUSBBandwidth, value);
        }
        status |= callParamCallbacks();
        return (asynStatus)status;
    }

    if (function == ADUSBBandwidthAuto) {
        ASI_BOOL autoMode = (value != 0) ? ASI_TRUE : ASI_FALSE;
        if (cameraID >= 0) {
            long bandwidthValue;
            ASI_BOOL isAuto = ASI_FALSE;
            ASIGetControlValue(cameraID, ASI_BANDWIDTHOVERLOAD, &bandwidthValue,
                               &isAuto);
            status |= ASISetControlValue(cameraID, ASI_BANDWIDTHOVERLOAD,
                                         bandwidthValue, autoMode);
        }
        status |= setIntegerParam(ADUSBBandwidthAuto, (int)autoMode);
        status |= callParamCallbacks();
        return (asynStatus)status;
    }

    status |= ADDriver::writeInt32(pasynUser, value);
    return (asynStatus)status;
}

asynStatus ZWODriver::writeFloat64(asynUser *pasynUser, epicsFloat64 value) {
    int function = pasynUser->reason;
    int status = asynSuccess;

    if (function == ADAcquireTime) {
        long exposureTime =
            clampUnsignedRange((long)(value * 1000.0 * 1000.0),
                               this->controlLimits.minExposure,
                               this->controlLimits.maxExposure);
        value = (epicsFloat64)exposureTime / 1000.0 / 1000.0;
        if (cameraID >= 0) {
            status |= ASISetControlValue(cameraID, ASI_EXPOSURE, exposureTime,
                                         ASI_FALSE);
        }
    } else if (function == ADGain) {
        value = (epicsFloat64)clampUnsignedRange((long)value,
                                                 this->controlLimits.minGain,
                                                 this->controlLimits.maxGain);
        if (cameraID >= 0) {
            status |=
                ASISetControlValue(cameraID, ASI_GAIN, (long)value, ASI_FALSE);
        }
    } else if (function == ADOffset) {
        value = (epicsFloat64)clampUnsignedRange((long)value,
                                                 this->controlLimits.minOffset,
                                                 this->controlLimits.maxOffset);
        if (cameraID >= 0) {
            status |=
                ASISetControlValue(cameraID, ASI_OFFSET, (long)value, ASI_FALSE);
        }
    } else if (function == ADTemperature) {
        value = (epicsFloat64)clampSignedRange((long)value,
                                               this->controlLimits.minTemp,
                                               this->controlLimits.maxTemp);
        if (cameraID >= 0) {
            status |= ASISetControlValue(cameraID, ASI_TARGET_TEMP, value,
                                         ASI_FALSE);
        }
    }

    status |= ADDriver::writeFloat64(pasynUser, value);
    return (asynStatus)status;
}

asynStatus ZWODriver::setReverse(int reverseX, int reverseY) {
    int status = asynSuccess;
    int reverse;
    if (reverseX && reverseY) {
        reverse = ASI_FLIP_BOTH;
    } else if (reverseX && !reverseY) {
        reverse = ASI_FLIP_HORIZ;
    } else if (reverseY && !reverseX) {
        reverse = ASI_FLIP_VERT;
    } else {
        reverse = ASI_FLIP_NONE;
    }

    status |= ASISetControlValue(cameraID, ASI_FLIP, reverse, ASI_FALSE);
    return (asynStatus)status;
}

asynStatus ZWODriver::setROIFormat(ROIFormat_t *out) {
    int status = asynSuccess;
    int colorMode, dataType;
    ASI_IMG_TYPE imgType;

    status |= getIntegerParam(NDColorMode, &colorMode);
    status |= getIntegerParam(NDDataType, &dataType);

    int binX, binY, minX, minY, sizeX, sizeY, maxSizeX, maxSizeY;
    int imgWidth, imgHeight, imgBin, startX, startY;
    status |= getIntegerParam(ADMinX, &minX);
    status |= getIntegerParam(ADMinY, &minY);
    status |= getIntegerParam(ADSizeX, &sizeX);
    status |= getIntegerParam(ADSizeY, &sizeY);
    status |= getIntegerParam(ADBinX, &binX);
    status |= getIntegerParam(ADBinY, &binY);

    maxSizeX = cameraInfo.MaxWidth;
    maxSizeY = cameraInfo.MaxHeight;

    // Image Type (Color & Data Type)
    if ((colorMode == NDColorModeMono) && (dataType == NDUInt8)) {
        if (cameraInfo.IsColorCam) {
            imgType = ASI_IMG_Y8;
        } else {
            imgType = ASI_IMG_RAW8;
        }
    } else if ((colorMode == NDColorModeMono) && (dataType == NDUInt16)) {
        if (cameraInfo.IsColorCam)
            goto unsupportedMode;
        imgType = ASI_IMG_RAW16;
    } else if ((colorMode == NDColorModeRGB3) && (dataType == NDUInt8)) {
        if (!cameraInfo.IsColorCam)
            goto unsupportedMode;
        imgType = ASI_IMG_RGB24;
    } else if ((colorMode == NDColorModeBayer) && (dataType == NDUInt8)) {
        if (!cameraInfo.IsColorCam)
            goto unsupportedMode;
        imgType = ASI_IMG_RAW8;
    } else if ((colorMode == NDColorModeBayer) && (dataType == NDUInt16)) {
        if (!cameraInfo.IsColorCam)
            goto unsupportedMode;
        imgType = ASI_IMG_RAW16;
    } else {
    unsupportedMode:
        asynPrint(
            this->pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: error unsupported data type %d and/or color mode %d\n",
            driverName, __func__, dataType, colorMode);

        return asynError;
    }

    // ROI
    if (binX < 1) {
        binX = 1;
        status |= setIntegerParam(ADBinX, binX);
    }
    if (binY < 1) {
        binY = 1;
        status |= setIntegerParam(ADBinY, binY);
    }

    if (binX != binY) {
        // X and Y binning must be equal
        return asynError;
    }
    imgBin = binX;

    if (minX + sizeX > maxSizeX) {
        sizeX = maxSizeX - minX;
        status |= setIntegerParam(ADSizeX, sizeX);
    }
    if (minY + sizeY > maxSizeY) {
        sizeY = maxSizeY - minY;
        status |= setIntegerParam(ADSizeY, sizeY);
    }

    imgWidth = sizeX / binX;
    imgHeight = sizeY / binY;
    startX = minX / binX;
    startY = minY / binY;

    status |= setIntegerParam(NDArraySizeX, imgWidth);
    status |= setIntegerParam(NDArraySizeY, imgHeight);

    status |= ASISetROIFormat(cameraID, imgWidth, imgHeight, imgBin, imgType);
    status |= ASISetStartPos(cameraID, startX, startY);

    if (status == asynSuccess && out != NULL) {
        out->colorMode = (NDColorMode_t)colorMode;
        out->dataType = (NDDataType_t)dataType;
        out->imgType = imgType;
        out->imgWidth = imgWidth;
        out->imgHeight = imgHeight;
        out->imgBin = imgBin;
        out->startX = startX;
        out->startY = startY;
    }

    callParamCallbacks();

    return ((asynStatus)status);
}

asynStatus ZWODriver::connectCamera() {
    int status = asynSuccess;
    ASI_ERROR_CODE asiStatus;

    int numConnectedCameras = ASIGetNumOfConnectedCameras();
    if (numConnectedCameras == 0) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: No ASI camera connected.\n", driverName, __func__);
        setConnectionState(false, "No ASI camera connected");
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
        setConnectionState(false, "Failed to open camera");
        return asynError;
    }

    asiStatus = ASIInitCamera(cameraID);
    if (asiStatus) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: Failed to init camera (id: %d, error: %d)\n",
                  driverName, __func__, cameraID, asiStatus);
        ASICloseCamera(cameraID);
        setConnectionState(false, "Failed to init camera");
        return asynError;
    }

    //
    ASIGetCameraPropertyByID(cameraID, &cameraInfo);
    this->cameraID = cameraID;
    this->cameraInfo = cameraInfo;
    memset(&this->controlLimits, 0, sizeof(this->controlLimits));

    // Print control capabilities
    int numControls;
    ASIGetNumOfControls(cameraID, &numControls);

    for (int i = 0; i < numControls; i++) {
        ASI_CONTROL_CAPS caps;
        ASIGetControlCaps(cameraID, i, &caps);

        epicsStdoutPrintf("%s:%s: Control %s, %ld-%ld, default %ld\n",
                          driverName, __func__, caps.Name, caps.MinValue,
                          caps.MaxValue, caps.DefaultValue);

        if (caps.ControlType == ASI_GAIN) {
    this->controlLimits.minGain = caps.MinValue;
    this->controlLimits.maxGain = caps.MaxValue;
        } else if (caps.ControlType == ASI_OFFSET) {
    this->controlLimits.minOffset = caps.MinValue;
    this->controlLimits.maxOffset = caps.MaxValue;
        } else if (caps.ControlType == ASI_EXPOSURE) {
    this->controlLimits.minExposure = caps.MinValue;
    this->controlLimits.maxExposure = caps.MaxValue;
        } else if (caps.ControlType == ASI_TARGET_TEMP) {
            this->controlLimits.minTemp = caps.MinValue;
            this->controlLimits.maxTemp = caps.MaxValue;
        } else if (caps.ControlType == ASI_BANDWIDTHOVERLOAD) {
            this->controlLimits.minUSB = caps.MinValue;
            this->controlLimits.maxUSB = caps.MaxValue;
        }
    }

    // Set some initial values for various parameters
    status |= setStringParam(ADManufacturer, "ZWO");
    status |= setStringParam(ADModel, cameraInfo.Name);
    status |= setStringParam(ADSerialNumber, "N/A");
    status |= setStringParam(ADFirmwareVersion, "N/A");
    status |= setStringParam(NDDriverVersion, driverVersion);
    status |= setStringParam(ADSDKVersion, ASIGetSDKVersion());

    status |= setIntegerParam(ADSizeX, cameraInfo.MaxWidth);
    status |= setIntegerParam(ADSizeY, cameraInfo.MaxHeight);
    status |= setIntegerParam(ADMaxSizeX, cameraInfo.MaxWidth);
    status |= setIntegerParam(ADMaxSizeY, cameraInfo.MaxHeight);
    status |= setIntegerParam(NDArraySizeX, cameraInfo.MaxWidth);
    status |= setIntegerParam(NDArraySizeY, cameraInfo.MaxHeight);

    status |= setDoubleParam(ADSensorPixelSize, cameraInfo.PixelSize);

    long bandwidthValue;
    ASI_BOOL isAuto = ASI_FALSE;
    ASIGetControlValue(cameraID, ASI_BANDWIDTHOVERLOAD, &bandwidthValue, &isAuto);

    status |= setIntegerParam(ADUSBBandwidth, bandwidthValue);
    status |= setIntegerParam(ADUSBBandwidthAuto, (int)isAuto);

    status |= applyCachedSettingsToCamera();
    callParamCallbacks();

    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: unable to set camera parameters on camera %d\n",
                  driverName, __func__, cameraID);
        disconnectCamera("Failed to configure camera");
        return asynError;
    }

    // Print ID
    // It turns out that this ID is only available to read after connecting to
    // the camera
    ASI_ID asiId;
    ASIGetID(cameraInfo.CameraID, &asiId);

    char idString[32] = {0};
    sprintf(idString, "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X", asiId.id[0],
            asiId.id[1], asiId.id[2], asiId.id[3], asiId.id[4], asiId.id[5],
            asiId.id[6], asiId.id[7]);

    epicsStdoutPrintf("%s:%s: Did connect to camera \"%s\" with id %d (%s)\n",
                      driverName, __func__, cameraInfo.Name,
                      cameraInfo.CameraID, idString);

    clearPendingStopEvent();
    return setConnectionState(true, "Idle");
}

asynStatus ZWODriver::disconnectCamera(const char *statusMessage) {
    int connectedCameraID = cameraID;
    cameraID = -1;

    stopEvent->signal();

    if (connectedCameraID >= 0) {
        ASIStopExposure(connectedCameraID);
        ASICloseCamera(connectedCameraID);
    }

    memset(&this->cameraInfo, 0, sizeof(this->cameraInfo));
    memset(&this->controlLimits, 0, sizeof(this->controlLimits));

    return setConnectionState(false, statusMessage);
}

void ZWODriver::captureTask() {
    int status = asynSuccess;
    int imageCounter;
    int numImages, numImagesCounter;
    int imageMode;
    int acquire = 0;
    int arrayCallbacks;
    double acquireTime;
    double timeRemaining = 0.0;
    epicsTimeStamp startTime, endTime, currentTime;
    double acquirePeriod;

    ASI_EXPOSURE_STATUS exposureStatus;
    ASI_ERROR_CODE asiStatus;
    ROIFormat_t roiFormat;

    this->lock();
    while (true) {
        if (cameraID < 0) {
            this->unlock();
            epicsThreadSleep(1);
            this->lock();
            continue;
        }

        // If not currently acquiring, wait for semaphore signal
        if (!acquire) {
            this->unlock();
            bool signal = this->startEvent->wait(1);
            this->lock();

            if (!signal)
                continue;
            acquire = 1;
            clearPendingStopEvent();
            setIntegerParam(ADNumImagesCounter, 0);
        }

        getDoubleParam(ADAcquireTime, &acquireTime);

        // Send parameters to camera
        status = asynSuccess;
        status |= setROIFormat(&roiFormat);

        int reverseX, reverseY;
        status |= getIntegerParam(ADReverseX, &reverseX);
        status |= getIntegerParam(ADReverseY, &reverseY);
        status |= setReverse(reverseX, reverseY);

        if (status != 0) {
            acquire = 0;
            setIntegerParam(ADAcquire, 0);
            setIntegerParam(ADStatus, ADStatusError);
            callParamCallbacks();
            continue;
        }

        // Wait until camera is ready to start with exposure
        this->unlock();
        asiStatus = ASI_SUCCESS;
        while (cameraID >= 0) {
            asiStatus = ASIGetExpStatus(cameraID, &exposureStatus);
            if ((asiStatus != ASI_SUCCESS) ||
                (exposureStatus != ASI_EXP_WORKING)) {
                break;
            }
            epicsThreadSleep(SHORT_WAIT);
        }
        this->lock();

        if (cameraID < 0) {
            acquire = 0;
            continue;
        }

        if (asiStatus != ASI_SUCCESS) {
            bool didReconnect = false;
            if (handleCameraError("checking exposure status", asiStatus,
                                  &didReconnect) == asynSuccess &&
                didReconnect) {
                continue;
            }
            acquire = 0;
            continue;
        }

        epicsTimeGetCurrent(&startTime);

        asiStatus = ASIStartExposure(cameraID, ASI_FALSE);
        if (asiStatus != ASI_SUCCESS) {
            bool didReconnect = false;
            if (handleCameraError("starting exposure", asiStatus,
                                  &didReconnect) == asynSuccess &&
                didReconnect) {
                continue;
            }
            acquire = 0;
            continue;
        }

        setStringParam(ADStatusMessage, "Waiting for exposure");
        setIntegerParam(ADStatus, ADStatusAcquire);
        callParamCallbacks();

        epicsTimeStamp lastUpdate;
        epicsTimeGetCurrent(&lastUpdate);

        // Wait until image has been acquired
        asiStatus = ASI_SUCCESS;
        while (cameraID >= 0) {
            asiStatus = ASIGetExpStatus(cameraID, &exposureStatus);
            if ((asiStatus != ASI_SUCCESS) ||
                (exposureStatus != ASI_EXP_WORKING)) {
                break;
            }

            epicsTimeGetCurrent(&currentTime);
            double sinceLastUpdate = epicsTimeDiffInSeconds(&currentTime, &lastUpdate);

            // Lower the update frequency
            if (sinceLastUpdate >= 0.01) { // update every 0.01 seconds
                timeRemaining = acquireTime - epicsTimeDiffInSeconds(&currentTime, &startTime);
                if (timeRemaining < 0) timeRemaining = 0;
                setDoubleParam(ADTimeRemaining, timeRemaining);
                callParamCallbacks();
                lastUpdate = currentTime;
            }

            setDoubleParam(ADTimeRemaining, timeRemaining);
            callParamCallbacks();

            this->unlock();
            bool s = this->stopEvent->wait(SHORT_WAIT);
            this->lock();
            if (s) {
                // Abort exposure
                if (cameraID >= 0) {
                    ASIStopExposure(cameraID);
                }

                acquire = 0;
                setIntegerParam(ADAcquire, 0);
                getIntegerParam(ADImageMode, &imageMode);
                if (imageMode == ADImageContinuous) {
                    setIntegerParam(ADStatus, ADStatusIdle);
                } else {
                    setIntegerParam(ADStatus, ADStatusAborted);
                }
                setDoubleParam(ADTimeRemaining, 0);
                callParamCallbacks();
                exposureStatus = ASI_EXP_FAILED;
                break;
            }
        }

        if (acquire == 0) {
            continue;
        }

        if (cameraID < 0) {
            acquire = 0;
            continue;
        }

        if (asiStatus != ASI_SUCCESS) {
            bool didReconnect = false;
            if (handleCameraError("waiting for exposure", asiStatus,
                                  &didReconnect) == asynSuccess &&
                didReconnect) {
                continue;
            }
            acquire = 0;
            continue;
        }

        setDoubleParam(ADTimeRemaining, 0);
        callParamCallbacks();

        getIntegerParam(NDArrayCounter, &imageCounter);
        getIntegerParam(ADNumImages, &numImages);
        getIntegerParam(ADNumImagesCounter, &numImagesCounter);
        getIntegerParam(ADImageMode, &imageMode);
        getIntegerParam(NDArrayCallbacks, &arrayCallbacks);
        getDoubleParam(ADAcquirePeriod, &acquirePeriod);

        if (exposureStatus == ASI_EXP_SUCCESS) {
            // Update counters
            numImagesCounter++;
            imageCounter++;
            setIntegerParam(NDArrayCounter, imageCounter);
            setIntegerParam(ADNumImagesCounter, numImagesCounter);
            setStringParam(ADStatusMessage, "Transfering image");

            // Allocate pImage and read data from camera
            NDArray *pImage;

            if (roiFormat.imgType == ASI_IMG_RGB24) {
                size_t dims[3] = {(size_t)roiFormat.imgWidth,
                                  (size_t)roiFormat.imgHeight, 3};
                pImage = this->pNDArrayPool->alloc(3, dims, roiFormat.dataType,
                                                   0, NULL);
            } else {
                size_t dims[2] = {(size_t)roiFormat.imgWidth,
                                  (size_t)roiFormat.imgHeight};
                pImage = this->pNDArrayPool->alloc(2, dims, roiFormat.dataType,
                                                   0, NULL);
            }

            pImage->uniqueId = imageCounter;
            pImage->timeStamp = startTime.secPastEpoch + startTime.nsec / 1.e9;
            updateTimeStamp(&pImage->epicsTS);

            asiStatus = ASIGetDataAfterExp(cameraID,
                                           (unsigned char *)pImage->pData,
                                           pImage->dataSize);
            if (asiStatus != ASI_SUCCESS) {
                pImage->release();
                bool didReconnect = false;
                if (handleCameraError("reading image data", asiStatus,
                                      &didReconnect) == asynSuccess &&
                    didReconnect) {
                    continue;
                }
                acquire = 0;
                continue;
            }

            setIntegerParam(NDArraySize, pImage->dataSize);

            this->getAttributes(pImage->pAttributeList);
            
            if (arrayCallbacks) {
                doCallbacksGenericPointer(pImage, NDArrayData, 0);
            }
            pImage->release();
        } else {
            bool didReconnect = false;
            if (checkCameraConnection("probing camera after failed exposure",
                                      &didReconnect) != asynSuccess) {
                acquire = 0;
                continue;
            }
            if (didReconnect) {
                continue;
            }

            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s:%s: Exposure failed with status %d\n", driverName,
                      __func__, exposureStatus);
            setStringParam(ADStatusMessage, "Exposure failed");
            setIntegerParam(ADStatus, ADStatusError);
        }

        callParamCallbacks();

        // Check if we are done with acquisition
        getIntegerParam(ADAcquire, &acquire);
        if ((acquire == 0) || (imageMode == ADImageSingle) ||
            ((imageMode == ADImageMultiple) &&
             (numImagesCounter >= numImages))) {
            acquire = 0;
            setIntegerParam(ADAcquire, 0);
            setIntegerParam(ADStatus, ADStatusIdle);
            callParamCallbacks();
        }

        if (acquire) {
            epicsTimeGetCurrent(&endTime);
            double elapsedTime = epicsTimeDiffInSeconds(&endTime, &startTime);
            double delay = acquirePeriod - elapsedTime;

            if (delay > 0) {
                setIntegerParam(ADStatus, ADStatusWaiting);
                callParamCallbacks();
                this->unlock();
                bool s = this->stopEvent->wait(delay);
                this->lock();
                if (s) {
                    acquire = 0;
                    if (imageMode == ADImageContinuous) {
                        setIntegerParam(ADStatus, ADStatusIdle);
                    } else {
                        setIntegerParam(ADStatus, ADStatusAborted);
                    }
                    setIntegerParam(ADAcquire, 0);
                    callParamCallbacks();
                }
            }
        }
    }
}

void ZWODriver::pollingTask() {
    epicsFloat64 timeout = 1;

    long cValue;
    ASI_BOOL cAuto;
    ASI_ERROR_CODE asiStatus;

    while (true) {
        epicsThreadSleep(timeout);
        if (cameraID < 0)
            continue;

        lock();

        asiStatus = ASIGetControlValue(cameraID, ASI_TEMPERATURE, &cValue,
                                       &cAuto);
        if (asiStatus != ASI_SUCCESS) {
            handleCameraError("polling temperature", asiStatus);
            unlock();
            continue;
        }
        double temperature = (double)(cValue) / 10.0;
        setDoubleParam(ADTemperatureActual, temperature);

        asiStatus = ASIGetControlValue(cameraID, ASI_COOLER_POWER_PERC,
                                       &cValue, &cAuto);
        if (asiStatus != ASI_SUCCESS) {
            handleCameraError("polling cooler power", asiStatus);
            unlock();
            continue;
        }
        setIntegerParam(ADCoolerPowerPerc, (int)cValue);

        asiStatus = ASIGetControlValue(cameraID, ASI_BANDWIDTHOVERLOAD,
                                       &cValue, &cAuto);
        if (asiStatus != ASI_SUCCESS) {
            handleCameraError("polling USB bandwidth", asiStatus);
            unlock();
            continue;
        }
        setIntegerParam(ADUSBBandwidth, (int)cValue);
        setIntegerParam(ADUSBBandwidthAuto, (int)cAuto);

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
