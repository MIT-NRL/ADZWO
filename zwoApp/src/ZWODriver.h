#ifndef ZWODRIVER_H
#define ZWODRIVER_H

#include "ADDriver.h"
#include "ASICamera2.h"
#include "NDArray.h"
#include "NDAttribute.h"
#include "asynDriver.h"
#include <epicsEvent.h>

#define SHORT_WAIT (0.00025)

#define ADOffsetString "OFFSET"
#define ADCoolerPowerPercString "COOLER_POWER_PERC"
#define ADSensorPixelSizeString "SENSOR_PIXEL_SIZE"
#define ADUSBBandwidthString "USB_BANDWIDTH"
#define ADUSBBandwidthAutoString "USB_BANDWIDTH_AUTO"
#define ADCameraConnectString "CAMERA_CONNECT"
#define ADHighSpeedModeString "HIGH_SPEED_MODE"
#define ADVideoModeString "VIDEO_MODE"

typedef struct ROIFormat {
    NDColorMode_t colorMode;
    NDDataType_t dataType;
    ASI_IMG_TYPE imgType;

    long imgWidth, imgHeight;
    long imgBin;
    long startX, startY;
} ROIFormat_t;

typedef struct _ASI_CONTROL_LIMITS {
    unsigned long minExposure;
    unsigned long maxExposure;
    unsigned long minGain;
    unsigned long maxGain;
    unsigned long minOffset;
    unsigned long maxOffset;
    unsigned long minUSB;
    unsigned long maxUSB;

    long minTemp;
    long maxTemp;
} ASI_CONTROL_LIMITS;

class ZWODriver : public ADDriver {
public:
    /**
     * \param[in] portName The name of the asyn port driver to be created.
     * \param[in] maxBuffers The maximum number of NDArray buffers that the
     *    NDArrayPool for this driver is allowed to allocate. Set this to -1 to
     *    allow an unlimited number of buffers.
     * \param[in] maxMemory The maximum amount of memory that the NDArrayPool
     * for this driver is allowed to allocate. Set this to -1 to allow an
     * unlimited amount of memory. \param[in] priority The thread priority for
     * the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
     * \param[in] stackSize The stack size for
     *    the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
     */
    ZWODriver(const char *portName, int maxBuffers, size_t maxMemory,
              int priority, int stackSize);
    ~ZWODriver();

    virtual asynStatus connect(asynUser *pasynUser);
    virtual asynStatus disconnect(asynUser *pasynUser);

    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);

    void captureTask();
    void pollingTask();

private:
    int cameraID;
    bool hasHighSpeedMode;
    ASI_CAMERA_INFO cameraInfo;
    ASI_CONTROL_LIMITS controlLimits;

    epicsEvent *startEvent;
    epicsEvent *stopEvent;

    void clearPendingStopEvent();
    asynStatus setConnectionState(bool connected, const char *statusMessage);
    asynStatus attemptReconnectOnce(const char *reason,
                                    bool *didReconnect = NULL);
    asynStatus handleCameraError(const char *operation,
                                 ASI_ERROR_CODE asiStatus,
                                 bool *didReconnect = NULL);
    asynStatus checkCameraConnection(const char *operation,
                                     bool *didReconnect = NULL);
    asynStatus applyCachedSettingsToCamera();
    asynStatus normalizeROI(ROIFormat_t *out, bool applyToCamera);
    asynStatus setROIFormat(ROIFormat_t *out);
    asynStatus connectCamera();
    asynStatus disconnectCamera(const char *statusMessage = "Disconnected");
    asynStatus setReverse(int reverseX, int reverseY);
    int nearestSupportedBin(int requestedBin) const;

protected:
    int ADOffset;
    int ADCoolerPowerPerc;
    int ADSensorPixelSize;
    int ADUSBBandwidth;
    int ADUSBBandwidthAuto;
    int ADCameraConnect;
    int ADHighSpeedMode;
    int ADVideoMode;
};

#endif
