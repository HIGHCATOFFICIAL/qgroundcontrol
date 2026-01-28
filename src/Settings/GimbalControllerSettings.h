#pragma once

#include <QtQmlIntegration/QtQmlIntegration>

#include "SettingsGroup.h"

class GimbalControllerSettings : public SettingsGroup
{
    Q_OBJECT
    QML_ELEMENT
    QML_UNCREATABLE("")
public:
    GimbalControllerSettings(QObject* parent = nullptr);
    DEFINE_SETTING_NAME_GROUP()

    DEFINE_SETTINGFACT(EnableOnScreenControl)
    DEFINE_SETTINGFACT(ControlType)
    DEFINE_SETTINGFACT(CameraVFov)
    DEFINE_SETTINGFACT(CameraHFov)
    DEFINE_SETTINGFACT(CameraSlideSpeed)
    DEFINE_SETTINGFACT(showAzimuthIndicatorOnMap)
    DEFINE_SETTINGFACT(toolbarIndicatorShowAzimuth)
    DEFINE_SETTINGFACT(toolbarIndicatorShowAcquireReleaseControl)
    DEFINE_SETTINGFACT(joystickButtonsSpeed)
    DEFINE_SETTINGFACT(zoomMaxSpeed)
    DEFINE_SETTINGFACT(zoomMinSpeed)
    DEFINE_SETTINGFACT(joystickGimbalEnabled)
    DEFINE_SETTINGFACT(joystickGimbalShowUI)
    DEFINE_SETTINGFACT(joystickGimbalPitchAxisIndex)
    DEFINE_SETTINGFACT(joystickGimbalYawAxisIndex)
    DEFINE_SETTINGFACT(joystickGimbalDeadband)
    DEFINE_SETTINGFACT(joystickGimbalExpo)
    DEFINE_SETTINGFACT(joystickGimbalSmoothing)
    DEFINE_SETTINGFACT(joystickGimbalSendRateHz)
    DEFINE_SETTINGFACT(joystickGimbalPitchLimit)
    DEFINE_SETTINGFACT(joystickGimbalYawLimit)
};
