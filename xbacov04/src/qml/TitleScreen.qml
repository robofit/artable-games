//================================================================================
// BACHELOR THESIS - AUGMENTED REALITY FOR DESK GAMES
//================================================================================

/*
 * Filename:       TitleScreen.qml
 * Date:           21. 10. 2017
 * Subject:        IBP - Bachelor Thesis
 * Description:    QML file for title screen
 * Author:         Juraj Bačovčin
 *                 (xbacov04@stud.fit.vutbr.cz)
 */

import QtQuick 2.1
import QtQuick.Window 2.0

/* Main application window */
Window
{
    id: window
    visible: true

    property string difficulty
    property string players
    property string building
    property string timer

    property bool calibrating
    property int calibPointSize
    property int calibPointX
    property int calibPointY

    difficulty: "Medium"
    players: "1"
    building: "1st Edition"
    timer: "OFF"

    calibrating: false
    calibPointSize: 0
    calibPointX: 0
    calibPointY: 0

    width: 1280
    height: 720

    minimumWidth: 640
    minimumHeight: 360

    title: "RESCUERS"

    Component.onCompleted:
    {
        setX (Screen.width / 2 - width / 2)
        setY (Screen.height / 2 - height / 2)
        window.showMaximized ()
    }

    /* Function drawing white point with specific variable position */
    function drawPoint (pointSize, pointX, pointY)
    {
        window.calibrating = true
        window.calibPointSize = pointSize
        window.calibPointX = pointX
        window.calibPointY = pointY
    }

    /* Function used to stop calibration mode of title screen */
    function stopCalibration ()
    {
        window.calibrating = false
    }

    /* Function used to decide which action should be executed upon clicking specific button */
    function executeMenuButton (operation)
    {
        if (operation === "New Game") setupGameWindow ()

        if (operation === "Load Game") showErrorMessage ("Saving and loading is not implemented in this version.")

        if (operation === "Options") showSettings ()

        if (operation === "Instructions") Qt.openUrlExternally ("http://www.indieboardsandcards.com/fpfr.php")

        if (operation === "Exit Game") Qt.quit ()
    }

    /* Function creating specific error message */
    function showErrorMessage (errorMessage)
    {
        var component = Qt.createComponent ("ErrorMessage.qml")
        var messagebox = component.createObject (window)
        messagebox.message = errorMessage
    }

    /* Function creating settings screen on top of title screen */
    function showSettings ()
    {
        var component = Qt.createComponent ("SettingsScreen.qml")
        var settings = component.createObject (window)
    }

    /* Function creating game screen on top of title screen */
    function setupGameWindow ()
    {
        var component = Qt.createComponent ("GameScreen.qml")
        var settings = component.createObject (window)
    }

    /* Background image for title screen */
    Image
    {
        width: parent.width
        height: parent.height
        source: "../img/background.jpg"
    }

    /* Column consisting of menu buttons */
    Column
    {
        anchors.right: parent.right
        anchors.rightMargin: window.width / 15

        anchors.top: parent.top
        anchors.topMargin: window.height / 4

        spacing: window.height / 25

        /* Button for starting new game */
        MenuButton
        {
            operation: "New Game"
        }

        /* Button for loading game */
        // TODO: add proper functionality
        MenuButton
        {
            operation: "Load Game"
        }

        /* Button for showing settings screen */
        MenuButton
        {
            operation: "Options"
        }

        /* Button for showing instructions */
        // TODO: add more in-game instructions
        MenuButton
        {
            operation: "Instructions"
        }

        /* Button for exiting game */
        MenuButton
        {
            operation: "Exit Game"
        }
    }

    /* Black rectangle with white point representing calibration mode of title screen */
    Rectangle
    {
        visible: window.calibrating
        color: "black"

        width: parent.width
        height: parent.height

        /* Mouse area covering whole title screen in order to stop user from unintentionally clicking on anything besides calibration points */
        MouseArea
        {
            enabled: window.calibrating
            anchors.fill: parent
        }

        /* Shape representing calibration point */
        Rectangle
        {
            visible: window.calibrating
            color: "white"

            width: window.calibPointSize
            height: window.calibPointSize

            radius: width * 0.5

            x: window.calibPointX
            y: window.calibPointY
        }
    }
}

//--------------------------------------------------------------------------------
// End of file TitleScreen.qml
//--------------------------------------------------------------------------------
