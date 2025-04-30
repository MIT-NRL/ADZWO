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
static const char *driverVersion = "0.2.0";

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

    createParam(ADOffsetString, asynParamFloat64, &ADOffset);
    createParam(ADCoolerPowerPercString, asynParamInt32,
                &ADCoolerPowerPerc);
    createParam(ADSensorPixelSizeString, asynParamFloat64,
                &ADSensorPixelSize);
    createParam(ADUSBBandwidthString, asynParamInt32,
                &ADUSBBandwidth);

    printf("\n\n\n\n\n");

    int status = asynSuccess;

    this->startEvent = new epicsEvent();
    this->stopEvent = new epicsEvent();

    this->cameraID = -1;
    this->connect(this->pasynUserSelf);

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
    int reverseX, reverseY;

    int acquiring;
    getIntegerParam(ADAcquire, &acquiring);

    if (function == ADAcquire) {
        if (value == 1 && !acquiring) {
            startEvent->signal();
        }

        if (value == 0 && acquiring) {
            stopEvent->signal();
        }
    }

    if ((function == ADBinX) || (function == ADBinY)) {
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
        if (value > this->controlLimits.maxUSB)
            value = this->controlLimits.maxUSB;
        if (value < this->controlLimits.minUSB)
            value = this->controlLimits.minUSB;
        status |= ASISetControlValue(cameraID, ASI_BANDWIDTHOVERLOAD, value,
                                     ASI_FALSE);
        long bandwidthValue;
        ASI_BOOL isAuto = ASI_FALSE; // Declare a variable for the fourth argument
        ASIGetControlValue(cameraID, ASI_BANDWIDTHOVERLOAD, &bandwidthValue,
                           &isAuto);
        status |= setIntegerParam(ADUSBBandwidth, bandwidthValue);
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
        long exposureTime = value * 1000 * 1000;
        // Make sure exposure time cannot be negative or exceed max exposure
        if (exposureTime > this->controlLimits.maxExposure)
            exposureTime = this->controlLimits.maxExposure;
        if (exposureTime < this->controlLimits.minExposure)
            exposureTime = this->controlLimits.minExposure;
        status |=
            ASISetControlValue(cameraID, ASI_EXPOSURE, exposureTime, ASI_FALSE);
    } else if (function == ADGain) {
        // Make sure gain cannot be negative or exceed max gain
        if (value > this->controlLimits.maxGain)
            value = this->controlLimits.maxGain;
        if (value < this->controlLimits.minGain)
            value = this->controlLimits.minGain;
        status |=
            ASISetControlValue(cameraID, ASI_GAIN, (long)value, ASI_FALSE);
    } else if (function == ADOffset) {
        // Make sure offset cannot be negative or exceed max offset
        if (value > this->controlLimits.maxOffset)
            value = this->controlLimits.maxOffset;
        if (value < this->controlLimits.minOffset)
            value = this->controlLimits.minOffset;
        status |=
            ASISetControlValue(cameraID, ASI_OFFSET, (long)value, ASI_FALSE);
    } else if (function == ADTemperature) {
        // Make sure target temperature cannot be below min or above max
        if (value > this->controlLimits.maxTemp)
            value = this->controlLimits.maxTemp;
        if (value < this->controlLimits.minTemp)
            value = this->controlLimits.minTemp;
        status |= ASISetControlValue(cameraID, ASI_TARGET_TEMP, value,
                                     ASI_FALSE);
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
    int64_t dataSize = 0;

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
            printf("Min temp: %ld, max temp: %ld\n", this->controlLimits.minTemp,
                   this->controlLimits.maxTemp);
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
    ASI_BOOL isAuto = ASI_FALSE; // Declare a variable for the fourth argument
    ASIGetControlValue(cameraID, ASI_BANDWIDTHOVERLOAD, &bandwidthValue, &isAuto);
    printf("Bandwidth overload: %ld\n, isAuto: %d\n", bandwidthValue, isAuto);

    status |= setIntegerParam(ADUSBBandwidth, bandwidthValue);

    callParamCallbacks();

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
    int imageCounter;
    int numImages, numImagesCounter;
    int imageMode;
    int acquire = 0;
    int arrayCallbacks;
    double acquireTime;
    double timeRemaining;
    epicsTimeStamp startTime, endTime, currentTime;
    double acquirePeriod;

    ASI_EXPOSURE_STATUS exposureStatus;
    ROIFormat_t roiFormat;

    this->lock();
    while (true) {
        if (cameraID < 0) {
            epicsThreadSleep(1);
            continue;
        }

        // If not currently acquiring, wait for semaphore signal
        if (!acquire) {
            this->unlock();
            bool signal = this->startEvent->wait(1);
            if (!status) 
                setStringParam(ADStatusMessage, "Idle");
            this->lock();

            if (!signal)
                continue;
            acquire = 1;
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
        while (!ASIGetExpStatus(cameraID, &exposureStatus) &&
               exposureStatus == ASI_EXP_WORKING) {
            epicsThreadSleep(SHORT_WAIT);
        }
        this->lock();

        epicsTimeGetCurrent(&startTime);

        if (ASIStartExposure(cameraID, ASI_FALSE)) {
            // FAILED
            setIntegerParam(ADStatus, ADStatusError);
            setStringParam(ADStatusMessage, "Failed to start exposure");
            callParamCallbacks();
            continue;
        }
        

        setStringParam(ADStatusMessage, "Waiting for exposure");
        setIntegerParam(ADStatus, ADStatusAcquire);
        callParamCallbacks();

        epicsTimeStamp lastUpdate;
        epicsTimeGetCurrent(&lastUpdate);

        // Wait until image has been acquired
        while (!ASIGetExpStatus(cameraID, &exposureStatus) &&
               exposureStatus == ASI_EXP_WORKING) {
            
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
                ASIStopExposure(cameraID);

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
                continue;
            }
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

            ASIGetDataAfterExp(cameraID, (unsigned char *)pImage->pData,
                               pImage->dataSize);

            setIntegerParam(NDArraySize, pImage->dataSize);

            this->getAttributes(pImage->pAttributeList);
            
            if (arrayCallbacks) {
                doCallbacksGenericPointer(pImage, NDArrayData, 0);
            }
            pImage->release();
        } else {
            // ERROR
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s:%s: Exposure failed with status %d\n", driverName,
                      __func__, exposureStatus);

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

        if (ASIGetControlValue(cameraID, ASI_COOLER_POWER_PERC, &cValue,
                             &cAuto) == ASI_SUCCESS) {
            setIntegerParam(ADCoolerPowerPerc, (int)cValue);
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
