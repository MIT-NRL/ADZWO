#include <epicsStdlib.h>
#include <iocsh.h>

#include "prosilicaDriver.h" 

// EPICS includes 
#include <epicsTime.h>
#include <iocInit.h>
#include <epicsThread.h>
#include <epicsEvent.h>

// Database defines PVs 
#include "camera.db" 

int main(int argc,char *argv[])
{

    // Initialize camera driver & params
    ProsilicaCamera camera;
    camera.connect();

    // Initialize EPICS 
    iocsh(initIOC()); 

    // Register PVs from database
    databaseInit("camera.db");

    // Start processing 
    processCamera();

    // Run till exit 
    runForever();

}

void processCamera()
{
   while(1) {

      // Set exposure from PV 
      camera.setExposure(exposureTimePV->get());

      // Grab image
      camera.grabImage();

      // Set timestamp PV
      imageTimestampPV->set(epicsTime);

      // Set frame rate PV
      frameRatePV->set(camera.frameRate());

      // Trigger image counter PV
      imageCounterPV->post();

      // Wait for next frame
      epicsEventSleep(camera.nextFrameEvent());
   }
}