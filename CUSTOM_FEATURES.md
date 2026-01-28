# Custom QGC Features

This document describes modifications made to the upstream QGroundControl codebase by our team. It serves as a reference for developers and helps track changes when merging upstream updates.

---

## Table of Contents

1. [Joystick Gimbal & Camera Calibration](#1-joystick-gimbal--camera-calibration)
2. [Bug Fixes](#bug-fixes)
3. [Configuration Reference](#configuration-reference)
4. [Files Modified from Upstream](#files-modified-from-upstream)

---

## 1. Joystick Gimbal & Camera Calibration

### Description

Adds gimbal pitch and camera zoom axis calibration to the joystick calibration flow. This allows users to assign any joystick axis to control gimbal pitch and camera zoom through a guided calibration process.

### How It Works

During joystick calibration, after the standard flight control axes (throttle, yaw, roll, pitch), four additional calibration steps appear:

1. **Gimbal Pitch Up/Right**: Move the gimbal pitch axis to maximum position
2. **Gimbal Pitch Down/Left**: Move the gimbal pitch axis to minimum position
3. **Camera Zoom In/Right**: Move the camera zoom axis to maximum position
4. **Camera Zoom Out/Left**: Move the camera zoom axis to minimum position

The user can assign any available joystick axis to these functions by moving the desired axis during calibration.

### UI Elements

**Attitude Controls Section** (joystick mode only):
- **Gimbal Pitch** slider - shows current axis value
- **Camera Zoom** slider - shows current axis value

**Stick Display Area** (joystick mode only):
- Two horizontal slider visualizers above the stick circles:
  - **Left slider**: Camera Zoom indicator
  - **Right slider**: Gimbal Pitch indicator
- During calibration, the slider indicators show the target position (left or right)

### Calibration Messages

| Step | Message |
|------|---------|
| Gimbal Pitch Up | "Move the Gimbal Pitch axis all the way up (or right) and hold it there..." |
| Gimbal Pitch Down | "Move the Gimbal Pitch axis all the way down (or left) and hold it there..." |
| Camera Zoom In | "Move the Camera Zoom axis all the way to zoom in (or right) and hold it there..." |
| Camera Zoom Out | "Move the Camera Zoom axis all the way to zoom out (or left) and hold it there..." |

### Controller Properties

| Property | Type | Description |
|----------|------|-------------|
| `gimbalPitchChannelMapped` | bool | Whether gimbal pitch axis is calibrated |
| `cameraZoomChannelMapped` | bool | Whether camera zoom axis is calibrated |
| `adjustedGimbalPitchChannelValue` | int | Current gimbal pitch axis value |
| `adjustedCameraZoomChannelValue` | int | Current camera zoom axis value |
| `gimbalPitchSliderPosition` | int | Target position for gimbal slider (-1, 0, 1) |
| `cameraZoomSliderPosition` | int | Target position for zoom slider (-1, 0, 1) |

### Notes

- Calibration steps only appear in **joystick mode** (not RC calibration)
- The `gimbalYawFunction` was renamed to `cameraZoomFunction` in the Joystick class
- Axis functions are saved with joystick calibration data

---

## Bug Fixes

### SettingsFact Value Display Fix

**Issue**: Settings with default values displayed as "-" or "-.----" instead of their actual values.

**Root Cause**: In `SettingsFact.cc`, when a setting's value was directly assigned to `_rawValue` (bypassing the normal setter), the `_rawValueIsNotSet` flag remained `true`, causing the UI to display placeholder text.

**Fix**: Added `_rawValueIsNotSet = false;` after direct assignment in the `SettingsFact` constructor.

**File**: `src/FactSystem/SettingsFact.cc`

**Affected upstream commit**: This appears to be an upstream bug introduced when the `_rawValueIsNotSet` flag was added to the `Fact` class.

---

## Configuration Reference

### Debug Logging

Enable gimbal controller debug logging:
```bash
export QT_LOGGING_RULES="Gimbal.GimbalController.debug=true"
```

---

## Files Modified from Upstream

This section helps identify merge conflicts when updating from upstream QGC.

### Modified Files

| File | Change Type | Description |
|------|-------------|-------------|
| `src/Joystick/Joystick.h` | Changed enum | Renamed `gimbalYawFunction` to `cameraZoomFunction` |
| `src/Joystick/Joystick.cc` | Added mappings | Gimbal pitch and camera zoom axis function mappings |
| `src/Vehicle/VehicleSetup/JoystickConfigController.cc` | Updated | Include gimbal/camera functions in calibration |
| `src/Vehicle/VehicleSetup/RemoteControlCalibrationController.h` | Added | Gimbal/camera calibration properties and signals |
| `src/Vehicle/VehicleSetup/RemoteControlCalibrationController.cc` | Added | Calibration steps, messages, state machine entries |
| `src/Vehicle/VehicleSetup/RemoteControlCalibration.qml` | Added UI | Gimbal/camera sliders and slider visualizers |
| `src/FactSystem/SettingsFact.cc` | Bug fix | Set `_rawValueIsNotSet = false` after direct assignment |

### Merge Strategy

When merging upstream updates:

1. **Low conflict risk**: JSON settings additions
2. **Medium conflict risk**: `.h` and `.cc` files with added methods
3. **Higher conflict risk**: QML files, CMakeLists.txt

For QML files, check if upstream has modified the same components. The gimbal-related changes are mostly additive and isolated to specific sections.

---

## Version History

| Date | Change | Author |
|------|--------|--------|
| 2025-01-28 | Added joystick gimbal pitch and camera zoom calibration | - |
| 2025-01-28 | Fixed SettingsFact value display bug | - |

---

## Contact

For questions about these customizations, contact the development team.
