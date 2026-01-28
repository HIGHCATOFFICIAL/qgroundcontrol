import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import QGroundControl
import QGroundControl.Controls
import QGroundControl.FactControls

ColumnLayout {
    spacing: ScreenTools.defaultFontPixelHeight / 2

    property var _gimbalControllerSettings: QGroundControl.settingsManager.gimbalControllerSettings
    property var _activeVehicle:            QGroundControl.multiVehicleManager.activeVehicle
    property var _gimbalController:         _activeVehicle ? _activeVehicle.gimbalController : null

    QGCPalette { id: qgcPal; colorGroupEnabled: true }

    SettingsGroupLayout {
        Layout.fillWidth:   true
        heading:            qsTr("Joystick Gimbal Control")
        showDividers:       false

        FactCheckBoxSlider {
            id:                 enableJoystickGimbalCheckbox
            Layout.fillWidth:   true
            text:               qsTr("Enable joystick gimbal control via GIMBAL_DEVICE_SET_ATTITUDE")
            fact:               _gimbalControllerSettings.joystickGimbalEnabled
        }

        QGCLabel {
            Layout.fillWidth:   true
            wrapMode:           Text.WordWrap
            font.pointSize:     ScreenTools.smallFontPointSize
            text:               qsTr("When enabled, the configured joystick axes will send absolute gimbal angle commands using the GIMBAL_DEVICE_SET_ATTITUDE MAVLink message.")
            visible:            !enableJoystickGimbalCheckbox.checked
        }
    }

    SettingsGroupLayout {
        Layout.fillWidth:   true
        heading:            qsTr("Axis Configuration")
        showDividers:       false
        visible:            enableJoystickGimbalCheckbox.checked

        LabelledFactTextField {
            Layout.fillWidth:   true
            label:              qsTr("Pitch Axis Index")
            fact:               _gimbalControllerSettings.joystickGimbalPitchAxisIndex
        }

        LabelledFactTextField {
            Layout.fillWidth:   true
            label:              qsTr("Yaw Axis Index")
            fact:               _gimbalControllerSettings.joystickGimbalYawAxisIndex
        }

        QGCLabel {
            Layout.fillWidth:   true
            wrapMode:           Text.WordWrap
            font.pointSize:     ScreenTools.smallFontPointSize
            text:               qsTr("Axis indices are 0-based. Common mappings: Axis 2 = right stick vertical, Axis 5 = right stick horizontal (varies by controller).")
        }
    }

    SettingsGroupLayout {
        Layout.fillWidth:   true
        heading:            qsTr("Input Processing")
        showDividers:       false
        visible:            enableJoystickGimbalCheckbox.checked

        LabelledFactTextField {
            Layout.fillWidth:   true
            label:              qsTr("Deadband (0-1)")
            fact:               _gimbalControllerSettings.joystickGimbalDeadband
        }

        LabelledFactTextField {
            Layout.fillWidth:   true
            label:              qsTr("Expo (0-1)")
            fact:               _gimbalControllerSettings.joystickGimbalExpo
        }

        LabelledFactTextField {
            Layout.fillWidth:   true
            label:              qsTr("Smoothing Alpha (0-1)")
            fact:               _gimbalControllerSettings.joystickGimbalSmoothing
        }

        QGCLabel {
            Layout.fillWidth:   true
            wrapMode:           Text.WordWrap
            font.pointSize:     ScreenTools.smallFontPointSize
            text:               qsTr("Deadband ignores small inputs. Expo adds curve (higher = more sensitive at extremes). Smoothing: 0 = max smooth, 1 = no smoothing.")
        }
    }

    SettingsGroupLayout {
        Layout.fillWidth:   true
        heading:            qsTr("Output Configuration")
        showDividers:       false
        visible:            enableJoystickGimbalCheckbox.checked

        LabelledFactTextField {
            Layout.fillWidth:   true
            label:              qsTr("Send Rate (Hz)")
            fact:               _gimbalControllerSettings.joystickGimbalSendRateHz
        }

        LabelledFactTextField {
            Layout.fillWidth:   true
            label:              qsTr("Pitch Limit (degrees)")
            fact:               _gimbalControllerSettings.joystickGimbalPitchLimit
        }

        LabelledFactTextField {
            Layout.fillWidth:   true
            label:              qsTr("Yaw Limit (degrees)")
            fact:               _gimbalControllerSettings.joystickGimbalYawLimit
        }

        QGCLabel {
            Layout.fillWidth:   true
            wrapMode:           Text.WordWrap
            font.pointSize:     ScreenTools.smallFontPointSize
            text:               qsTr("Full stick deflection maps to the configured angle limits. Commands are sent at the specified rate when input is active.")
        }
    }

    SettingsGroupLayout {
        Layout.fillWidth:   true
        heading:            qsTr("MAVLink Message Log")
        showDividers:       false
        visible:            enableJoystickGimbalCheckbox.checked && _gimbalController

        RowLayout {
            Layout.fillWidth: true

            QGCLabel {
                text: qsTr("Recent GIMBAL_DEVICE_SET_ATTITUDE messages:")
            }

            Item { Layout.fillWidth: true }

            QGCButton {
                text: qsTr("Clear Log")
                onClicked: {
                    if (_gimbalController) {
                        _gimbalController.clearMessageLog()
                    }
                }
            }
        }

        Rectangle {
            Layout.fillWidth:       true
            Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 12
            color:                  qgcPal.windowShade
            border.color:           qgcPal.groupBorder
            border.width:           1

            QGCFlickable {
                id:                 logFlickable
                anchors.fill:       parent
                anchors.margins:    ScreenTools.defaultFontPixelWidth / 2
                contentHeight:      logColumn.height
                clip:               true

                ColumnLayout {
                    id:     logColumn
                    width:  parent.width
                    spacing: 2

                    Repeater {
                        model: _gimbalController ? _gimbalController.gimbalMessageLog : []

                        QGCLabel {
                            Layout.fillWidth:   true
                            text:               modelData
                            font.pointSize:     ScreenTools.smallFontPointSize
                            font.family:        ScreenTools.fixedFontFamily
                            wrapMode:           Text.WrapAnywhere
                        }
                    }

                    QGCLabel {
                        visible:            !_gimbalController || _gimbalController.gimbalMessageLog.length === 0
                        text:               qsTr("No messages sent yet. Move joystick axes to generate gimbal commands.")
                        font.pointSize:     ScreenTools.smallFontPointSize
                        color:              qgcPal.colorGrey
                        wrapMode:           Text.WordWrap
                    }
                }
            }
        }

        QGCLabel {
            Layout.fillWidth:   true
            wrapMode:           Text.WordWrap
            font.pointSize:     ScreenTools.smallFontPointSize
            text:               qsTr("Log shows quaternion values [w,x,y,z] and euler angles. Use MAVLink Inspector for detailed packet analysis.")
        }
    }

    Item {
        Layout.fillHeight: true
    }
}
