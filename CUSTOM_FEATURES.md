# Custom QGC Features

This document describes modifications made to the upstream QGroundControl codebase by our team. It serves as a reference for developers and helps track changes when merging upstream updates.

---

## Table of Contents

1. [Joystick Gimbal Control](#1-joystick-gimbal-control)
2. [Bug Fixes](#bug-fixes)
3. [Configuration Reference](#configuration-reference)
4. [Files Modified from Upstream](#files-modified-from-upstream)

---

## 1. Joystick Gimbal Control

### Description

Adds joystick-based gimbal control that reads axis inputs, applies preprocessing (deadband, exponential curves, smoothing), and sends `GIMBAL_DEVICE_SET_ATTITUDE` MAVLink messages for absolute angle positioning.

### How It Works

1. **Axis Input**: Reads raw joystick axis values (configurable, default: axis 2 for pitch, axis 5 for yaw)
2. **Deadband**: Eliminates small unintentional inputs near center position
3. **Exponential Curve**: Provides finer control near center, more aggressive response at extremes
4. **Smoothing**: Low-pass filter to reduce jitter and provide smooth gimbal movement
5. **Angle Mapping**: Maps processed input (-1 to +1) to configured angle limits
6. **MAVLink Output**: Sends `GIMBAL_DEVICE_SET_ATTITUDE` messages at configurable rate

### Feature Status

| Aspect | Default Value |
|--------|---------------|
| Feature enabled | `true` |
| UI visible | `false` (developer option) |

### Enabling/Disabling

**Enable the feature** (sends MAVLink commands):
- Setting: `joystickGimbalEnabled`
- Default: `true` (enabled)
- Location: Gimbal Indicator popup → Joystick Gimbal Control section (when UI visible)

**Show/Hide the UI** (developer option):
- Setting: `joystickGimbalShowUI`
- Default: `false` (hidden)
- To enable: Set to `true` in QGC settings or via Settings Override file

When `joystickGimbalShowUI` is `true`, the UI appears in:
- **Gimbal Indicator popup** (toolbar) → "Joystick Gimbal Control" section with MAVLink message log
- **Vehicle Setup → Joystick** → "Gimbal" tab

### Settings

| Setting | Type | Default | Range | Description |
|---------|------|---------|-------|-------------|
| `joystickGimbalEnabled` | bool | `true` | - | Enable/disable joystick gimbal control |
| `joystickGimbalShowUI` | bool | `false` | - | Show joystick gimbal UI (developer option) |
| `joystickGimbalPitchAxisIndex` | uint32 | `2` | 0-7 | Joystick axis index for pitch control |
| `joystickGimbalYawAxisIndex` | uint32 | `5` | 0-7 | Joystick axis index for yaw control |
| `joystickGimbalDeadband` | double | `0.06` | 0.0-0.5 | Deadband threshold |
| `joystickGimbalExpo` | double | `0.20` | 0.0-1.0 | Exponential curve factor (0=linear, 1=cubic) |
| `joystickGimbalSmoothing` | double | `0.20` | 0.01-1.0 | Smoothing alpha (lower=smoother, higher=responsive) |
| `joystickGimbalSendRateHz` | uint32 | `50` | 1-100 | MAVLink message send rate in Hz |
| `joystickGimbalPitchLimit` | double | `30.0` | 1.0-90.0 | Maximum pitch angle in degrees |
| `joystickGimbalYawLimit` | double | `90.0` | 1.0-180.0 | Maximum yaw angle in degrees |

### Algorithm Details

```
Input Processing Pipeline:
    raw_axis → deadband → expo → smoothing → angle_mapping → quaternion → MAVLink

Deadband:
    if |x| <= deadband: return 0
    else: return sign(x) * (|x| - deadband) / (1 - deadband)

Expo:
    output = (1 - expo) * x + expo * x³

Smoothing (low-pass filter):
    smoothed = alpha * new_value + (1 - alpha) * previous_smoothed
```

### MAVLink Message

Sends `GIMBAL_DEVICE_SET_ATTITUDE` (message ID 284) with:
- Quaternion representing desired pitch/yaw angles (roll = 0)
- Flags: `GIMBAL_DEVICE_FLAGS_ROLL_LOCK | GIMBAL_DEVICE_FLAGS_PITCH_LOCK | GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME`
- Target: Active gimbal's manager component ID

### Thread Safety

The joystick runs in a separate thread. Input is marshalled to the main Qt thread via `Qt::QueuedConnection` signal/slot pattern to ensure thread-safe access to settings and MAVLink sending.

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

### Settings Override File

To enable hidden features or override defaults, create a Settings Override JSON file. Example for enabling gimbal UI:

```json
{
    "GimbalControllerSettings": {
        "joystickGimbalShowUI": true
    }
}
```

### Debug Logging

Enable gimbal controller debug logging:
```bash
export QT_LOGGING_RULES="Gimbal.GimbalController.debug=true"
```

---

## Files Modified from Upstream

This section helps identify merge conflicts when updating from upstream QGC.

### New Files

| File | Description |
|------|-------------|
| `src/Vehicle/VehicleSetup/JoystickComponentGimbal.qml` | Gimbal settings UI for Joystick configuration |
| `CUSTOM_FEATURES.md` | This documentation file |

### Modified Files

| File | Change Type | Description |
|------|-------------|-------------|
| `src/Settings/GimbalController.SettingsGroup.json` | Added settings | 10 new joystick gimbal settings |
| `src/Settings/GimbalControllerSettings.h` | Added macros | `DEFINE_SETTINGFACT` for new settings |
| `src/Settings/GimbalControllerSettings.cc` | Added macros | `DECLARE_SETTINGSFACT` for new settings |
| `src/Gimbal/GimbalController.h` | Added members | Joystick gimbal methods, signals, slots, timer, state variables |
| `src/Gimbal/GimbalController.cc` | Added implementation | Joystick input processing, MAVLink sending, message logging |
| `src/Joystick/Joystick.cc` | Added call | Forwards axis values to `GimbalController::processJoystickGimbalInput()` |
| `src/UI/toolbar/GimbalIndicator.qml` | Added UI sections | "Joystick Gimbal Control" and "MAVLink Message Log" sections |
| `src/Vehicle/VehicleSetup/JoystickComponent.qml` | Added tab | "Gimbal" tab (visibility controlled by setting) |
| `src/Vehicle/VehicleSetup/CMakeLists.txt` | Added file | `JoystickComponentGimbal.qml` to QML_FILES |
| `src/FactSystem/SettingsFact.cc` | Bug fix | Set `_rawValueIsNotSet = false` after direct assignment |

### Merge Strategy

When merging upstream updates:

1. **Low conflict risk**: New files, JSON settings additions
2. **Medium conflict risk**: `.h` and `.cc` files with added methods
3. **Higher conflict risk**: QML files, CMakeLists.txt

For QML files, check if upstream has modified the same components. The gimbal-related changes are mostly additive and isolated to specific sections.

---

## Version History

| Date | Change | Author |
|------|--------|--------|
| 2025-01-28 | Initial joystick gimbal control feature | - |
| 2025-01-28 | Added joystickGimbalShowUI flag to hide UI by default | - |
| 2025-01-28 | Fixed SettingsFact value display bug | - |

---

## Contact

For questions about these customizations, contact the development team.
