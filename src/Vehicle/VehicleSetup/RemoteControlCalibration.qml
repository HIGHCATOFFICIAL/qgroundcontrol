import QtQuick
import QtQuick.Controls
import QtQuick.Dialogs
import QtQuick.Layouts

import QGroundControl
import QGroundControl.FactControls
import QGroundControl.Controls

/// Base class for Remote Control Calibration (supports both RC and Joystick)
ColumnLayout {
    required property var controller
    property Component additionalSetupComponent
    property Component additionalMonitorComponent

    // Controllers need access to these UI elements
    property alias statusText: statusText
    property alias cancelButton: cancelButton
    property alias nextButton: nextButton

    id: root
    spacing: ScreenTools.defaultFontPixelHeight

    property bool useDeadband: false

    property real _channelValueDisplayWidth: ScreenTools.defaultFontPixelWidth * 30
    property bool _deadbandActive: useDeadband

    QGCPalette { id: qgcPal; colorGroupEnabled: root.enabled }

    RowLayout {
        // Left Column - Attitude Controls display
        ColumnLayout {
            Layout.alignment: Qt.AlignTop
            spacing: ScreenTools.defaultFontPixelHeight

            ColumnLayout {
                id: attitudeControlsLayout
                Layout.fillWidth: true
                spacing: ScreenTools.defaultFontPixelHeight

                QGCLabel { text: qsTr("Attitude Controls") }

                RowLayout {
                    Layout.fillWidth: true

                    QGCLabel {
                        Layout.fillWidth: true
                        text: qsTr("Roll")
                    }

                    RemoteControlChannelValueDisplay {
                        Layout.preferredWidth: root._channelValueDisplayWidth
                        mode: RemoteControlChannelValueDisplay.MappedValue
                        channelValueMin: controller.channelValueMin
                        channelValueMax: controller.channelValueMax
                        channelMapped: controller.rollChannelMapped
                        channelValue: controller.adjustedRollChannelValue
                        deadbandValue: controller.rollDeadband
                        deadbandEnabled: root._deadbandActive
                    }
                }

                RowLayout {
                    Layout.fillWidth: true

                    QGCLabel {
                        Layout.fillWidth: true
                        text: qsTr("Pitch")
                    }

                    RemoteControlChannelValueDisplay {
                        Layout.preferredWidth: root._channelValueDisplayWidth
                        mode: RemoteControlChannelValueDisplay.MappedValue
                        channelValueMin: controller.channelValueMin
                        channelValueMax: controller.channelValueMax
                        channelMapped: controller.pitchChannelMapped
                        channelValue: controller.adjustedPitchChannelValue
                        deadbandValue: controller.pitchDeadband
                        deadbandEnabled: root._deadbandActive
                    }
                }

                RowLayout {
                    Layout.fillWidth: true

                    QGCLabel {
                        Layout.fillWidth: true
                        text: qsTr("Yaw")
                    }

                    RemoteControlChannelValueDisplay {
                        Layout.preferredWidth: root._channelValueDisplayWidth
                        mode: RemoteControlChannelValueDisplay.MappedValue
                        channelValueMin: controller.channelValueMin
                        channelValueMax: controller.channelValueMax
                        channelMapped: controller.yawChannelMapped
                        channelValue: controller.adjustedYawChannelValue
                        deadbandValue: controller.yawDeadband
                        deadbandEnabled: root._deadbandActive
                    }
                }

                RowLayout {
                    Layout.fillWidth: true

                    QGCLabel {
                        Layout.fillWidth: true
                        text: qsTr("Throttle")
                    }

                    RemoteControlChannelValueDisplay {
                        Layout.preferredWidth: root._channelValueDisplayWidth
                        mode: RemoteControlChannelValueDisplay.MappedValue
                        channelValueMin: controller.channelValueMin
                        channelValueMax: controller.channelValueMax
                        channelMapped: controller.throttleChannelMapped
                        channelValue: controller.adjustedThrottleChannelValue
                        deadbandValue: controller.throttleDeadband
                        deadbandEnabled: root._deadbandActive
                    }
                }

                // Gimbal/Camera controls (joystick mode only)
                RowLayout {
                    Layout.fillWidth: true
                    visible: controller.joystickMode

                    QGCLabel {
                        Layout.fillWidth: true
                        text: qsTr("Gimbal Pitch")
                    }

                    RemoteControlChannelValueDisplay {
                        Layout.preferredWidth: root._channelValueDisplayWidth
                        mode: RemoteControlChannelValueDisplay.MappedValue
                        channelValueMin: controller.channelValueMin
                        channelValueMax: controller.channelValueMax
                        channelMapped: controller.gimbalPitchChannelMapped
                        channelValue: controller.adjustedGimbalPitchChannelValue
                        deadbandValue: controller.gimbalPitchDeadband
                        deadbandEnabled: root._deadbandActive
                    }
                }

                RowLayout {
                    Layout.fillWidth: true
                    visible: controller.joystickMode

                    QGCLabel {
                        Layout.fillWidth: true
                        text: qsTr("Camera Zoom")
                    }

                    RemoteControlChannelValueDisplay {
                        Layout.preferredWidth: root._channelValueDisplayWidth
                        mode: RemoteControlChannelValueDisplay.MappedValue
                        channelValueMin: controller.channelValueMin
                        channelValueMax: controller.channelValueMax
                        channelMapped: controller.cameraZoomChannelMapped
                        channelValue: controller.adjustedCameraZoomChannelValue
                        deadbandValue: controller.cameraZoomDeadband
                        deadbandEnabled: root._deadbandActive
                    }
                }
            }
        }

        // Right Column - Stick Display
        ColumnLayout {
            Layout.alignment: Qt.AlignTop
            spacing: ScreenTools.defaultFontPixelHeight / 2

            Rectangle {
                id: stickDisplayContainer
                implicitWidth: stickDisplayLayout.width + _margins * 2
                implicitHeight: stickDisplayLayout.height + _margins * 2
                border.color: qgcPal.text
                border.width: 1
                color: qgcPal.window
                radius: ScreenTools.defaultBorderRadius

                property real _margins: ScreenTools.defaultFontPixelHeight / 2
                property real _stickAdjust: leftStickDisplay.width / 2 - _margins * 1.25

                ColumnLayout {
                    id: stickDisplayLayout
                    anchors.leftMargin: stickDisplayContainer._margins
                    anchors.topMargin: stickDisplayContainer._margins
                    anchors.left: parent.left
                    anchors.top: parent.top
                    spacing: stickDisplayContainer._margins

                    RowLayout {
                        spacing: ScreenTools.defaultFontPixelWidth * 2

                        QGCComboBox {
                            id: transmitterModeComboBox
                            model: [ qsTr("Mode 1"), qsTr("Mode 2"), qsTr("Mode 3"), qsTr("Mode 4") ]
                            enabled: !controller.calibrating

                            onActivated: (index) => controller.transmitterMode = index + 1

                            Component.onCompleted: currentIndex = controller.transmitterMode - 1
                        }

                        QGCCheckBox {
                            id: centeredThrottleCheckBox
                            text: qsTr("Centered Throttle")
                            checked: controller.centeredThrottle
                            enabled: !controller.calibrating
                            visible: !controller.joystickMode

                            onClicked: controller.centeredThrottle = checked
                        }
                    }

                    // Slider displays for gimbal pitch and camera zoom (above stick circles, joystick mode only)
                    RowLayout {
                        Layout.fillWidth: true
                        spacing: stickDisplayContainer._margins * 2
                        visible: controller.joystickMode

                        // Left slider - Camera Zoom
                        Item {
                            id: cameraZoomSliderContainer
                            implicitWidth: leftStickDisplay.implicitWidth
                            implicitHeight: ScreenTools.defaultFontPixelHeight * 1.5

                            property real sliderTrackWidth: width * 0.8
                            property real sliderAdjust: (sliderTrackWidth / 2) - (ScreenTools.defaultFontPixelHeight * 0.6 / 2)

                            // Slider track
                            Rectangle {
                                anchors.centerIn: parent
                                width: parent.sliderTrackWidth
                                height: 1
                                color: qgcPal.buttonHighlight

                                // Left end marker
                                Rectangle {
                                    anchors.left: parent.left
                                    anchors.verticalCenter: parent.verticalCenter
                                    width: 1
                                    height: ScreenTools.defaultFontPixelHeight * 0.5
                                    color: qgcPal.buttonHighlight
                                }

                                // Right end marker
                                Rectangle {
                                    anchors.right: parent.right
                                    anchors.verticalCenter: parent.verticalCenter
                                    width: 1
                                    height: ScreenTools.defaultFontPixelHeight * 0.5
                                    color: qgcPal.buttonHighlight
                                }

                                // Center marker
                                Rectangle {
                                    anchors.horizontalCenter: parent.horizontalCenter
                                    anchors.verticalCenter: parent.verticalCenter
                                    width: 1
                                    height: ScreenTools.defaultFontPixelHeight * 0.3
                                    color: qgcPal.buttonHighlight
                                }

                                // Position indicator (filled circle)
                                Rectangle {
                                    x: parent.width / 2 + cameraZoomSliderContainer.sliderAdjust * controller.cameraZoomSliderPosition - width / 2
                                    anchors.verticalCenter: parent.verticalCenter
                                    width: ScreenTools.defaultFontPixelHeight * 0.6
                                    height: width
                                    radius: width / 2
                                    color: qgcPal.buttonHighlight
                                }
                            }
                        }

                        // Right slider - Gimbal Pitch
                        Item {
                            id: gimbalPitchSliderContainer
                            implicitWidth: leftStickDisplay.implicitWidth
                            implicitHeight: ScreenTools.defaultFontPixelHeight * 1.5

                            property real sliderTrackWidth: width * 0.8
                            property real sliderAdjust: (sliderTrackWidth / 2) - (ScreenTools.defaultFontPixelHeight * 0.6 / 2)

                            // Slider track
                            Rectangle {
                                anchors.centerIn: parent
                                width: parent.sliderTrackWidth
                                height: 1
                                color: qgcPal.buttonHighlight

                                // Left end marker
                                Rectangle {
                                    anchors.left: parent.left
                                    anchors.verticalCenter: parent.verticalCenter
                                    width: 1
                                    height: ScreenTools.defaultFontPixelHeight * 0.5
                                    color: qgcPal.buttonHighlight
                                }

                                // Right end marker
                                Rectangle {
                                    anchors.right: parent.right
                                    anchors.verticalCenter: parent.verticalCenter
                                    width: 1
                                    height: ScreenTools.defaultFontPixelHeight * 0.5
                                    color: qgcPal.buttonHighlight
                                }

                                // Center marker
                                Rectangle {
                                    anchors.horizontalCenter: parent.horizontalCenter
                                    anchors.verticalCenter: parent.verticalCenter
                                    width: 1
                                    height: ScreenTools.defaultFontPixelHeight * 0.3
                                    color: qgcPal.buttonHighlight
                                }

                                // Position indicator (filled circle)
                                Rectangle {
                                    x: parent.width / 2 + gimbalPitchSliderContainer.sliderAdjust * controller.gimbalPitchSliderPosition - width / 2
                                    anchors.verticalCenter: parent.verticalCenter
                                    width: ScreenTools.defaultFontPixelHeight * 0.6
                                    height: width
                                    radius: width / 2
                                    color: qgcPal.buttonHighlight
                                }
                            }
                        }
                    }

                    // Stick displays (circles)
                    RowLayout {
                        Layout.fillWidth: true
                        spacing: stickDisplayContainer._margins * 2

                        Rectangle {
                            id: leftStickDisplay
                            Layout.alignment: Qt.AlignLeft
                            implicitWidth: ScreenTools.defaultFontPixelHeight * 5
                            implicitHeight: implicitWidth
                            radius: implicitWidth / 2
                            border.color: qgcPal.buttonHighlight
                            border.width: 1
                            color: qgcPal.window

                            Rectangle {
                                x: parent.width / 2 + stickDisplayContainer._stickAdjust * controller.stickDisplayPositions[0] - width / 2
                                y: parent.height / 2 + stickDisplayContainer._stickAdjust * -controller.stickDisplayPositions[1] - height / 2
                                width: ScreenTools.defaultFontPixelHeight
                                height: width
                                radius: width / 2
                                color: qgcPal.buttonHighlight
                            }
                        }

                        Rectangle {
                            Layout.alignment: Qt.AlignRight
                            implicitWidth: leftStickDisplay.implicitWidth
                            implicitHeight: implicitWidth
                            radius: implicitWidth / 2
                            border.color: qgcPal.buttonHighlight
                            border.width: 1
                            color: qgcPal.window

                            Rectangle {
                                x: parent.width / 2 + stickDisplayContainer._stickAdjust * controller.stickDisplayPositions[2] - width / 2
                                y: parent.height / 2 + stickDisplayContainer._stickAdjust * -controller.stickDisplayPositions[3] - height / 2
                                width: ScreenTools.defaultFontPixelHeight
                                height: width
                                radius: width / 2
                                color: qgcPal.buttonHighlight
                            }
                        }
                    }
                }
            }
        }
    }

    // Command Buttons and Status Text
    RowLayout {
        Layout.preferredWidth: parent.width
        spacing: ScreenTools.defaultFontPixelWidth

        QGCButton {
            id: cancelButton
            text: qsTr("Cancel")
            onClicked: controller.cancelButtonClicked()
        }

        QGCButton {
            id: nextButton
            primary: true
            text: qsTr("Calibrate")

            onClicked: {
                if (text === qsTr("Calibrate")) {
                    if (controller.channelCount < controller.minChannelCount) {
                        mainWindow.showMessageDialog(qsTr("Remote Not Ready"),
                                                        controller.channelCount == 0 ? qsTr("Please turn on remote.") :
                                                                                    (controller.channelCount < controller.minChannelCount ?
                                                                                            qsTr("%1 channels or more are needed to fly.").arg(controller.minChannelCount) :
                                                                                            qsTr("Ready to calibrate.")))
                        return
                    } else if (!controller.joystickMode) {
                        mainWindow.showMessageDialog(qsTr("Zero Trims"),
                                                        qsTr("Before calibrating you should zero all your trims and subtrims. Click Ok to start Calibration.\n\n%1").arg(
                                                            (QGroundControl.multiVehicleManager.activeVehicle.px4Firmware ? "" : qsTr("Please ensure all motor power is disconnected AND all props are removed from the vehicle."))),
                                                        Dialog.Ok,
                                                        function() { controller.nextButtonClicked() })
                        return
                    }
                }
                controller.nextButtonClicked()
            }
        }

        QGCLabel {
            id: statusText
            Layout.fillWidth: true
            wrapMode: Text.WordWrap
        }
    }

    Rectangle {
        id: separator
        Layout.fillWidth: true
        implicitHeight: 1
        color: qgcPal.text
    }

    // Additional Setup + Channel Monitor
    RowLayout {
        Layout.fillWidth: true
        spacing: ScreenTools.defaultFontPixelHeight


        Item {
            Layout.fillWidth: true
            implicitHeight: 1
            visible: additionalSetupComponent === undefined
        }

        Loader {
            id: additionalSetupLoader
            Layout.alignment: Qt.AlignTop
            sourceComponent: additionalSetupComponent
        }

        ColumnLayout {
            Layout.alignment: Qt.AlignTop
            Layout.fillWidth: true
            spacing: ScreenTools.defaultFontPixelHeight

            RemoteControlChannelMonitor {
                id: channelMonitor
                Layout.fillWidth: true
                twoColumn: false
                channelCount: controller.channelCount
                channelValueMin: controller.channelValueMin
                channelValueMax: controller.channelValueMax

                Connections {
                    target: controller
                    onRawChannelValueChanged: (channel, channelValue) => channelMonitor.rawChannelValueChanged(channel, channelValue)
                }
            }

            Loader {
                id: additionalMonitorLoader
                Layout.preferredWidth: parent.width
                sourceComponent: additionalMonitorComponent
            }
        }
    }
}
