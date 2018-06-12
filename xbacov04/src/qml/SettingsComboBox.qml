//================================================================================
// BACHELOR THESIS - AUGMENTED REALITY FOR DESK GAMES
//================================================================================

/*
 * Filename:       SettingsComboBox.qml
 * Date:           18. 11. 2017
 * Subject:        IBP - Bachelor Thesis
 * Description:    QML file for combo box
 * Author:         Juraj Bačovčin
 *                 (xbacov04@stud.fit.vutbr.cz)
 */

import QtQuick 2.1
import "../javascript/theme.js" as Theme

/* Main rectangle representing combo box */
Rectangle
{
    id: combobox

    property string title
	
    property variant items
    property alias selectedItem: chosenText.text
    property alias selectedIndex: listView.currentIndex

    signal comboboxClicked

    width: settings.width / 3
    height: settings.height / 12

    items: [""]
    title: "Undefined:"

    /* Text displaying label of combo box */
    Text
    {
        text: title

        anchors.bottom: parent.top
        anchors.bottomMargin: parent.width / 30

        anchors.horizontalCenter: parent.horizontalCenter

        font.family: "Arial"
        font.bold: true
        font.pointSize: (window.height + window.width) / 120

        color: Theme.button
    }

    /* Rectangle representing chosen item of combo box */
    Rectangle
    {
        id: chosen

        width: parent.width
        height: combobox.height

        radius: 4
        color: Theme.combo_box

        /* Text representing chosen option */
        Text
        {
            id: chosenText
            text: combobox.items [0]

            anchors.left: parent.left
            anchors.leftMargin: parent.width / 20

            anchors.verticalCenter: parent.verticalCenter

            font.family: "Arial"
            font.bold: true
            font.pointSize: (window.height + window.width) / 140

            color: Theme.button
        }

        /* Mouse area unpacking all possible options of combo box upon clicking */
        MouseArea
        {
            anchors.fill: parent

            onClicked:
            {
                combobox.state = combobox.state === "dropdown" ? "dropped" : "dropdown"
            }
        }
    }

    /* Rectangle representing area for possible combo box options */
    Rectangle
    {
        id: dropdown

        width: combobox.width
        height: 0

        clip: true
        radius: 4

        anchors.top: chosen.bottom
        anchors.margins: 2

        /* List of possible options of combo box */
        ListView
        {
            id: listView

            height: settings.height

            model: combobox.items
            currentIndex: 0

            delegate: Item
            {
                width: combobox.width
                height: combobox.height

                /* Rectangle representing specific options of combo box */
                Rectangle
                {
                    id: selection

                    width: parent.width
                    height: parent.height

                    color: selectionArea.containsMouse ? Theme.combo_box : Theme.combo_box_dropdown
                }

                /* Text displaying specific options of combo box */
                Text
                {
                    text: modelData

                    anchors.left: parent.left
                    anchors.leftMargin: parent.width / 20

                    anchors.verticalCenter: parent.verticalCenter

                    font.family: "Arial"
                    font.bold: true
                    font.pointSize: (window.height + window.width) / 200

                    color: Theme.button
                }

                /* Mouse area selecting specific options of combo box upon clicking */
                MouseArea
                {
                    id: selectionArea

                    hoverEnabled: true
                    anchors.fill: parent

                    onClicked:
                    {
                        combobox.state = "dropped"

                        var prevSelection = chosenText.text
                        chosenText.text = modelData

                        if (chosenText.text != prevSelection) combobox.comboboxClicked ()

                        listView.currentIndex = index
                    }
                }
            }
        }
    }

    states: State
    {
        name: "dropdown"

        /* Change of combo box area height */
        PropertyChanges
        {
            target: dropdown
            height: combobox.height * combobox.items.length
        }
    }

    transitions: Transition
    {
        /* Simple animation of unpacking all possible combo box options */
        NumberAnimation
        {
            target: dropdown
            properties: "height"
            easing.type: Easing.OutExpo
            duration: 500
        }
    }
}

//--------------------------------------------------------------------------------
// End of file SettingsComboBox.qml
//--------------------------------------------------------------------------------
