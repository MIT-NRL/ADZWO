# Changelog

All notable changes to `ADZWO` are documented in this file.

This project now uses semantic versioning.

## [1.0.0] - 2026-03-31

First formal `ADZWO` release.

### Added

- camera connect and disconnect PVs that work without restarting the IOC
- startup support with the camera disconnected
- reconnect handling after camera power cycles
- single-attempt automatic reconnect handling in selected acquisition failure paths
- `VideoMode` support using the ZWO SDK streaming API
- `HighSpeedMode` support for cameras that expose the SDK control
- USB bandwidth auto/manual support and readback
- macOS SDK packaging for `darwin-x86` and `darwin-aarch64`
- a cleaned-up standalone example IOC in [iocs/zwoIOC](/home/sfayfar/GitHub/ADZWO/iocs/zwoIOC)
- expanded project documentation and startup instructions

### Changed

- updated bundled `ASICamera2` SDK files to `1.41`
- improved SDK binary installation rules in [zwoSupport/Makefile](/home/sfayfar/GitHub/ADZWO/zwoSupport/Makefile)
- improved driver behavior when the camera disappears during acquisition
- improved offline parameter caching and reconnect reapplication
- updated PyDM screen to expose the new driver features and reorganized them around readout vs collect behavior
- moved manual camera connection handling to the dedicated `CameraConnect` PV rather than relying on the asyn port connect state
- normalized ROI and binning values on write and applied supported values back to the readbacks immediately when possible

### Fixed

- reconnect now works after a camera power cycle without requiring an IOC restart
- startup with the camera absent no longer requires the driver port to be torn down
- first-acquire-after-reconnect behavior
- video-mode disconnect detection
- standalone IOC build and boot configuration
- late-connect behavior after IOC startup with the camera absent
- autosave installation and request-file coverage for ZWO-specific settings
- `NDArraySize` / `ArraySize_RBV` now reports the actual current image payload rather than the size of a reused pool buffer
- UI issues including misplaced controls, dark-mode header readability, and stale static-label tooltips

## [0.3.0]

This release established the last pre-1.0 driver version that was reported by the IOC.

### Added

- time-remaining reporting during acquisition
- Linux `asi.rules` packaging for non-root camera access
- cooler power reporting and camera control limit handling
- sensor pixel-size metadata and related UI display
- USB bandwidth control and USB auto-bandwidth mode

### Changed

- updated bundled ZWO SDK files to `1.37`
- unified and expanded the operator UI
- refined ROI and image size calculations

### Fixed

- bandwidth control handling
- image width and image size calculations
- time-remaining update behavior

## [0.1.1]

Follow-up release focused on usability and restore behavior.

### Added

- offset control
- image reverse controls

### Fixed

- image data size calculation
- save and restore behavior

Historical note:

- git tag `R0-1-1` corresponds to this release

## [0.1.0]

First feature-complete early driver release.

### Added

- basic ZWO driver and IOC structure
- SDK documentation packaging
- ROI support
- autosave support
- continuous and multiple acquisition modes
- initial PyDM control screen
- attribute list update callback support

## [pre-0.1.0]

Initial repository setup and early development before the first versioned release.
