//================================================================================
// BACHELOR THESIS - AUGMENTED REALITY FOR DESK GAMES
//================================================================================

/*
 * Filename:       SettingsScreen.qml
 * Date:           18. 11. 2017
 * Subject:        IBP - Bachelor Thesis
 * Description:    QML file for settings screen
 * Author:         Juraj Bačovčin
 *                 (xbacov04@stud.fit.vutbr.cz)
 */

import QtQuick 2.1
import "../javascript/theme.js" as Theme

/* White rectangle representing settings screen */
Rectangle
{
    id: settings
    visible: true

    border.color: Theme.button
    border.width: (window.height + window.width) / 600
    radius: (window.height + window.width) / 150

    width: window.width / 1.1
    height: window.height / 1.45
    color: Theme.border

    anchors.horizontalCenter: parent.horizontalCenter

    anchors.top: parent.top
    anchors.topMargin: window.height / 4

    /* Function applying selected options and returning to title screen */
    function applySettings ()
    {
        window.difficulty = difficulty.selectedItem
        window.players = players.selectedItem
        window.building = building.selectedItem
        window.timer = timer.selectedItem
        settings.destroy ()
    }

    /* Mouse area covering settings screen in order to stop user from unintentionally clicking on title screen */
    MouseArea
    {
        anchors.fill: parent
    }

    /* Row consisting of some options */
    Row
    {
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.verticalCenter: parent.verticalCenter

        spacing: settings.width / 6

        /* Combo box representing setting of used building */
        SettingsComboBox
        {
            id: building
            title: "Choose Building:"
            selectedItem: window.building
            items: ["1st Edition", "2nd Edition"]
        }

        /* Combo box representing setting of timer */
        // TODO: add possibility of its deduction in order to end turn in case of player not acting on time
        SettingsComboBox
        {
            id: timer
            title: "Turn Timer ON/OFF:"
            selectedItem: window.timer
            items: ["ON", "OFF"]
        }
    }

    /* Row consisting of other options */
    Row
    {
        anchors.horizontalCenter: parent.horizontalCenter

        anchors.bottom: parent.bottom
        anchors.bottomMargin: parent.height / 1.3

        spacing: settings.width / 6

        /* Combo box representing setting of difficulty */
        SettingsComboBox
        {
            id: difficulty
            title: "Select Difficulty:"
            selectedItem: window.difficulty
            items: ["Easy", "Medium", "Hard"]
        }

        /* Combo box representing setting of number of players */
        SettingsComboBox
        {
            id: players
            title: "Number of Players:"
            selectedItem: window.players
            items: ["1", "2", "3", "4", "5", "6"]
        }
    }

    /* Row consisting of menu buttons */
    Row
    {
        anchors.horizontalCenter: parent.horizontalCenter

        anchors.top: parent.top
        anchors.topMargin: parent.height / 1.25

        spacing: settings.width / 2

        /* Button for returning to title screen without saving selected options */
        MenuButton
        {
            id: cancel
            operation: "Cancel"

            /* Mouse area destroying settings screen without saving selected options upon clicking */
            MouseArea
            {
                anchors.fill: parent

                onPressed: cancel.pressed = true
                onReleased: cancel.pressed = false
                onClicked: settings.destroy ()
            }
        }

        /* Button for saving selected options and returning to title screen */
        MenuButton
        {
            id: confirm
            operation: "Confirm"

            /* Mouse area applying selected options and destroying settings screen upon clicking */
            MouseArea
            {
                anchors.fill: parent

                onPressed: confirm.pressed = true
                onReleased: confirm.pressed = false
                onClicked: applySettings ()
            }
        }
    }
}

//--------------------------------------------------------------------------------
// End of file SettingsScreen.qml
//--------------------------------------------------------------------------------
