# ADZWO

`ADZWO` is an [EPICS](https://epics-controls.org) [areaDetector](https://areadetector.github.io/areaDetector/index.html) driver for [ZWO ASI astronomy cameras](https://www.zwoastro.com/) using ZWO's `ASICamera2` SDK.

Version: `1.0.0`

The runtime driver version reported through `$(P)$(R)DriverVersion_RBV` is defined in [ZWODriver.cpp](/home/sfayfar/GitHub/ADZWO/zwoApp/src/ZWODriver.cpp#L23) and matches this release. For release history and feature evolution, see [CHANGELOG.md](/home/sfayfar/GitHub/ADZWO/CHANGELOG.md).

## Features

- areaDetector `ADDriver` implementation for ZWO ASI cameras
- support for mono, Bayer, and RGB camera output modes
- `UInt8` and `UInt16` image acquisition
- standard exposure-mode acquisition and a separate ZWO video-mode acquisition path
- immediate ROI and binning normalization to camera-supported values
- reconnect handling that supports startup with the camera disconnected and recovery after power cycling
- bundled minimal IOC and PyDM screen for standalone testing

## Driver Functionality

Most operator interaction uses the normal areaDetector camera PVs from `ADBase`, plus a small set of ZWO-specific additions.

Standard areaDetector functionality supported by this driver includes:

- acquisition control with `Acquire`, `AcquireBusy`, `ImageMode`, `NumImages`, and `AcquirePeriod`
- exposure and timing control with `AcquireTime` and `TimeRemaining`
- camera signal controls with `Gain`
- cooling setpoint and temperature readback with `Temperature` and `TemperatureActual`
- image formatting with `ColorMode` and `DataType`
- ROI control with `MinX`, `MinY`, `SizeX`, and `SizeY`
- binning through `BinX` and `BinY`
- image reversal through `ReverseX` and `ReverseY`
- standard camera metadata readback such as `Model`, `MaxSizeX/Y`, `DriverVersion`, and `SDKVersion`

The driver maps those standard PVs onto ZWO SDK controls such as:

- exposure
- gain
- target temperature
- ROI and start position
- image format
- flip mode

The capture backends behave as follows:

- `VideoMode = Exposure` uses `ASIStartExposure` and `ASIGetDataAfterExp`
- `VideoMode = Video` uses `ASIStartVideoCapture` and `ASIGetVideoData`

That means the same areaDetector PVs are used in both cases, but the underlying SDK path changes depending on whether you want snapshot-style acquisition or higher-rate streaming.

ZWO-specific additions exposed by this module are defined in [ZWODriver.template](/home/sfayfar/GitHub/ADZWO/zwoApp/Db/ZWODriver.template):

- `CameraConnect`
- `HighSpeedMode`
- `VideoMode`
- `Offset`
- `CoolerPowerPerc_RBV`
- `SensorPixelSize_RBV`
- `USBBandwidth`
- `USBBandwidthAuto`

## Repository Layout

- [zwoApp/src](/home/sfayfar/GitHub/ADZWO/zwoApp/src): driver source
- [zwoApp/Db](/home/sfayfar/GitHub/ADZWO/zwoApp/Db): driver database and autosave request files
- [zwoApp/op/PyDM](/home/sfayfar/GitHub/ADZWO/zwoApp/op/PyDM): PyDM operator screen
- [zwoSupport](/home/sfayfar/GitHub/ADZWO/zwoSupport): bundled ZWO SDK headers, rules, and platform libraries
- [iocs/zwoIOC](/home/sfayfar/GitHub/ADZWO/iocs/zwoIOC): minimal standalone IOC for driver testing

## Requirements

- EPICS Base 7
- `asyn`
- `areaDetector` `ADCore`
- `busy`
- a C++ toolchain supported by your EPICS build
- a supported ZWO ASI camera

This repository is currently configured against the local support layout in [configure/RELEASE.local](/home/sfayfar/GitHub/ADZWO/configure/RELEASE.local). Update that file to match your site.

## SDK Support

The bundled SDK support files are installed from [zwoSupport](/home/sfayfar/GitHub/ADZWO/zwoSupport).

ZWO software and SDK downloads are available from:

- [ZWO Software Downloads](https://www.zwoastro.com/software/)
- [ZWO Astronomy Cameras](https://www.zwoastro.com/product-category/cameras/)
- [ZWO Main Website](https://www.zwoastro.com/)

Bundled SDK version:

- `ASICamera2` SDK `1.41`

Bundled binary platforms:

- Linux `x86_64`
- Linux `aarch64`
- Windows `x64`
- macOS Intel `darwin-x86`
- macOS Apple Silicon `darwin-aarch64`

The support makefile that installs these SDK files is [zwoSupport/Makefile](/home/sfayfar/GitHub/ADZWO/zwoSupport/Makefile).

## Linux Camera Access

For Linux systems, install the supplied udev rules file so the camera can be opened without running the IOC as root:

```bash
sudo install zwoSupport/asi.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Then reconnect the camera.

The SDK README in [zwoSupport/README.txt](/home/sfayfar/GitHub/ADZWO/zwoSupport/README.txt) also recommends checking the USB filesystem memory setting:

```bash
cat /sys/module/usbcore/parameters/usbfs_memory_mb
```

ZWO recommends a value of `200`.

## Build Instructions

### 1. Configure support paths

Edit [configure/RELEASE.local](/home/sfayfar/GitHub/ADZWO/configure/RELEASE.local) so that these variables point to your EPICS installation:

```make
EPICS_BASE=/path/to/epics-base
SUPPORT=/path/to/synApps/support
ASYN=$(SUPPORT)/asyn-R4-44-2
AREA_DETECTOR=$(SUPPORT)/areaDetector-R3-14
ADSUPPORT=$(AREA_DETECTOR)/ADSupport
ADCORE=$(AREA_DETECTOR)/ADCore
```

### 2. Build the driver module

From the top of the repository:

```bash
make -C /path/to/ADZWO
```

This builds:

- the SDK support install in [zwoSupport](/home/sfayfar/GitHub/ADZWO/zwoSupport)
- the driver library in [zwoApp/src](/home/sfayfar/GitHub/ADZWO/zwoApp/src)
- the database files in [zwoApp/Db](/home/sfayfar/GitHub/ADZWO/zwoApp/Db)

## Standalone Test IOC

The repository includes a minimal IOC that can be used to test the driver without a larger beamline IOC:

- IOC top: [iocs/zwoIOC](/home/sfayfar/GitHub/ADZWO/iocs/zwoIOC)
- boot directory: [iocZWODriver](/home/sfayfar/GitHub/ADZWO/iocs/zwoIOC/iocBoot/iocZWODriver)

### Build the standalone IOC

```bash
make -C /path/to/ADZWO/iocs/zwoIOC
```

### Start the standalone IOC

```bash
cd /path/to/ADZWO/iocs/zwoIOC/iocBoot/iocZWODriver
../../bin/linux-x86_64/ZWODriverApp st.cmd
```

The boot script is [st.cmd](/home/sfayfar/GitHub/ADZWO/iocs/zwoIOC/iocBoot/iocZWODriver/st.cmd).

The bundled test IOC is intentionally minimal:

- one ZWO camera driver instance
- one `NDStdArrays` plugin
- no site-specific autosave setup
- no PVA or extra common plugins

That makes it a good first-stop IOC when testing camera connection, image acquisition, reconnect behavior, or SDK changes.

## Configuring the Driver in Your Own IOC

Register the driver with:

```iocsh
ZWODriverConfig("PORT", -1, -1)
```

Then load the driver records:

```iocsh
dbLoadRecords("$(ADZWO)/db/ZWODriver.template",
              "P=PREFIX:,R=cam1:,PORT=PORT,ADDR=0,TIMEOUT=1")
```

Arguments to `ZWODriverConfig` are defined in [ZWODriver.cpp](/home/sfayfar/GitHub/ADZWO/zwoApp/src/ZWODriver.cpp#L1440):

1. `portName`
2. `maxBuffers`
3. `maxMemory`
4. `priority`
5. `stackSize`

For most setups, `-1, -1` is a good default for unlimited `NDArrayPool` buffers and memory.

## Startup and Operation

Typical startup sequence:

1. Build `ADZWO`.
2. Install the Linux `asi.rules` file if needed.
3. Start the standalone IOC or your site IOC.
4. Open the camera UI or use `caget` and `caput`.
5. Use `CameraConnect` to connect the camera if it is not already connected.
6. Set acquisition mode, exposure time, gain, and image format.
7. Start acquisition with `Acquire`.

Important behavior in this release:

- The IOC can start with the camera disconnected.
- The asyn port stays alive even when the hardware is absent.
- Camera disconnects are represented in driver status PVs rather than by tearing down the whole port.
- Power-cycle recovery is supported without restarting the IOC.
- A single reconnect attempt is made automatically in selected failure paths.
- `CameraConnect` is the intended operator PV for manual camera connect and disconnect. The usual asyn port connect state is not used as the primary hardware-presence indicator.

## ROI and Binning Behavior

The driver validates and normalizes ROI and binning values when they are written rather than waiting until the next exposure.

Practical effects:

- `BinX` and `BinY` are kept on a camera-supported bin factor
- `SizeX` is corrected to a multiple of `8` after binning
- `SizeY` is corrected to a multiple of `2` after binning
- `MinX` and `MinY` are aligned to the effective binning
- `Min + Size` is clamped so the ROI stays within the sensor

When the camera is connected and idle, the driver also applies the ROI immediately and reads the accepted values back from the SDK. That means unsupported starts or sizes can usually be verified and corrected without taking an exposure.

## Exposure Mode vs Video Mode

`ADZWO` supports two acquisition backends:

- `VideoMode = Exposure`
  Uses the ZWO exposure API. This is the normal snapshot-style areaDetector path.
- `VideoMode = Video`
  Uses the ZWO streaming API. This is the mode to use for higher frame-rate acquisition.

Practical notes:

- Full-frame maximum rate is much higher in `Video` mode than in the exposure path.
- ROI reduction increases frame rate significantly.
- Binning may not improve frame rate on all cameras if the underlying sensor path is still limited by readout or transfer.

## High Speed Mode

`HighSpeedMode` is a ZWO SDK control separate from `VideoMode`.

- `VideoMode` selects the acquisition backend.
- `HighSpeedMode` asks the camera to use its faster internal readout mode if the model supports it.

For cameras like the `ASI1600`, ZWO documents high-speed mode as a speed-versus-ADC-precision tradeoff. In practice:

- `UInt16` + normal mode is the best choice for dynamic range
- `UInt8` + video mode + high-speed mode is the best choice for maximum frame rate

## USB Bandwidth Control

The driver exposes both manual and auto USB bandwidth controls:

- `USBBandwidth`
- `USBBandwidth_RBV`
- `USBBandwidthAuto`
- `USBBandwidthAuto_RBV`

In auto mode, the readback reflects the SDK control value the camera reports. This is a control setting, not a live measured USB throughput monitor.

## PyDM Screen

The bundled PyDM screen is:

- [ADZWO.ui](/home/sfayfar/GitHub/ADZWO/zwoApp/op/PyDM/ADZWO.ui)

It includes controls for:

- standard areaDetector camera parameters
- camera connect and disconnect
- USB bandwidth and auto mode
- high-speed mode
- capture mode (`VideoMode`) in the collect section
- standard image plugin views and statistics panes

Notes:

- `HighSpeedMode` is shown with the readout controls because it changes the camera readout path rather than the acquisition sequence.
- `VideoMode` is shown as `Capture mode` in the collect section because it selects the SDK acquisition backend.
- The screen intentionally does not expose `TriggerMode` or `NumExposures`, because the current ZWO driver does not implement separate trigger handling or multi-exposure-per-image assembly.

## Troubleshooting

### IOC starts but no camera is present

This is supported. The driver should report disconnected status and allow a later manual connect.

### Linux permission error opening the camera

Install [asi.rules](/home/sfayfar/GitHub/ADZWO/zwoSupport/asi.rules) and reconnect the camera.

### Camera was power cycled

The driver should detect the disconnect, mark the camera offline, and try one reconnect in selected failure paths. If the camera does not come back automatically, use `CameraConnect` to reconnect. An IOC restart should not be required.

### ROI or binning values change after they are entered

This is expected when the requested values do not match the camera's ROI rules. The driver snaps the ROI to values the SDK accepts and updates the readbacks immediately when possible.


### Startup restore issues in a site IOC

If your site IOC restores PVs with autosave, make sure the ZWO request files are installed and included in your autosave configuration. The bundled standalone IOC is intentionally minimal and does not enable site autosave by default.

### Full-frame frame rate seems lower than vendor marketing numbers

Use `VideoMode = Video` and a reduced ROI when testing maximum frame rate. ZWO's published top-end rates are for the streaming/video path, not the snapshot exposure path.

## Versioning

`ADZWO` now uses semantic versioning.

- Current version: `1.0.0`
- SDK bundled in this release: `ASICamera2 1.41`
- Runtime driver version string: `1.0.0`

Detailed release history is in [CHANGELOG.md](/home/sfayfar/GitHub/ADZWO/CHANGELOG.md).

## Version History Summary

- `1.0.0`:
  First formal release. Includes reconnect handling, offline startup support, video mode, high-speed mode control, immediate ROI normalization, improved standalone IOC support, updated SDK packaging, PyDM cleanup, and expanded user documentation.
- `0.3.0`:
  Added USB bandwidth controls, cooler and sensor metadata, control limits, Linux rules packaging, UI refinements, and SDK `1.37`.
- `0.1.1`:
  Added offset and image reversing, and fixed restore behavior and data-size handling. Historical git tag: `R0-1-1`.
- `0.1.0`:
  First early feature release with ROI support, autosave support, continuous and multiple acquisition, and the initial PyDM control screen.
