//================================================================================
// BACHELOR THESIS - AUGMENTED REALITY FOR DESK GAMES
//================================================================================

/*
 * Filename:       GameScreen.qml
 * Date:           19. 11. 2017
 * Subject:        IBP - Bachelor Thesis
 * Description:    QML file for game screen
 * Author:         Juraj Bačovčin
 *                 (xbacov04@stud.fit.vutbr.cz)
 */

import QtQuick 2.1
import QtQuick.Window 2.0
import "../javascript/theme.js" as Theme
import "../javascript/gameboard.js" as GameBoard

/* Visible rectangle representing game screen */
Rectangle
{
    id: game

    visible: true
    enabled: true

    property int difficulty
    property int players
    property int building
    property bool timer

    property int dead
    property int saved
    property int damaged

    property var clock
    property int onMove
    property int currentAP
    property bool currentLoad
    property bool unloadable
    property int turn

    width: window.width
    height: window.height

    difficulty: window.difficulty === "Easy" ? 1 : window.difficulty === "Medium" ? 2 : 3
    players: parseInt (window.players)
    building: window.building === "1st Edition" ? 0 : 1
    timer: window.timer === "OFF" ? false : true

    dead: 0
    saved: 0
    damaged: 0

    clock: new Date (0, 0, 0, 0, 0, 0)
    onMove: 1
    currentAP: 1
    currentLoad: false
    unloadable: false
    turn: 0

    /* Function used to decide which action should be executed upon clicking specific button */
    function executeGameButton (operation)
    {
        if (operation === "Pause Game") pauseGame ()

        if (operation === "Save Game") showErrorMessage ("Saving and loading is not implemented in this version.")

        if (operation === "Exit Game") game.destroy()
    }

    /* Function actualizing timer */
    function actualizeTimer (actual_time, seconds)
    {
        if (actual_time.getHours () < 24) actual_time.setSeconds (actual_time.getSeconds () + seconds)

        return actual_time
    }

    /* Function moving actual player */
    function movePlayer (column, row)
    {
        GameBoard.moveCurrentPlayer (column, row)
    }

    /* Function executing any other player action other than movement */
    function playerAction (column, row)
    {
        if (column === -1 && row === -1) GameBoard.unloadAlert ()

        else GameBoard.playerAction (column, row)
    }

    /* Function changing current state of specific doors */
    function moveDoors (column, row, isLeft)
    {
        GameBoard.moveDoors (column, row, isLeft)
    }

    /* Function used for pausing and resuming game */
    function pauseGame ()
    {
        if (timer.running)
        {
            timer.stop ()
            pause_button.button_text = "Resume Game"
            gameboard.enabled = false
        }

        else
        {
            timer.start ()
            pause_button.button_text = "Pause Game"
            gameboard.enabled = true
        }
    }

    /* Function used for ending game */
    function gameOver (isWin)
    {
        timer.stop ()

        if (isWin) showErrorMessage ("Congratulations! You win!")

        else showErrorMessage ("Game over. You lose.")

        pause_button.enabled = false
        gameboard.enabled = false
    }

    /* Mouse area covering whole game screen in order to stop user from unintentionally clicking on title screen */
    MouseArea
    {
        anchors.fill: parent
    }

    /* Background image representing game map */
    Image
    {
        width: parent.width / 1.3
        height: parent.height
        source: game.building === 0 ? "../img/1st_edition_gameboard.jpg" : "../img/2nd_edition_gameboard.jpg"

        /* Rectangle representing gameboard */
        Rectangle
        {
            id: gameboard

            width: parent.width - parent.width / 9.8
            height: parent.height

            anchors.left: parent.left
            anchors.leftMargin: parent.width / 9.8

            color: "transparent"

            Component.onCompleted: GameBoard.startGame ()
        }
    }

    /* Special item allowing to spread fire faster used for testing */
    Item
    {
        focus: true

        Keys.onPressed:
        {
            if (event.key == Qt.Key_Space)
            {
                GameBoard.addSmoke ()
                GameBoard.checkAfterEffects ()
                GameBoard.enableAvailableSquares ()
            }
        }
    }

    /* In-game timer */
    Timer
    {
        id: timer
        interval: 1000
        running: true
        repeat: true
        onTriggered: game.clock = actualizeTimer (game.clock, 1)
    }

    /* Row consisting of one column filled with game information */
    Row
    {
        anchors.left: parent.left
        anchors.leftMargin: parent.width / 1.22

        anchors.top: parent.top
        anchors.topMargin: parent.height / 30

        /* Column consisting of game information */
        Column
        {
            spacing: game.height / 25

            /* Text displaying in-game timer */
            Text
            {
                font.bold: true
                font.pointSize: (game.height + game.width) / 120
                color: game.timer ? Theme.combo_box : Theme.button

                text: (game.clock).toLocaleTimeString ()
            }

            /* Health bar with label representing number of remaining deaths */
            HealthBar
            {
                range: 4
                lost: game.dead > range ? range : game.dead
                label: (range - lost) + " death(s) left"
                fromleft: false
            }

            /* Health bar with label representing house health */
            HealthBar
            {
                range: 24
                lost: game.damaged > range ? range : game.damaged
                label: 100 - Math.floor (lost / range * 100) + "% house health"
                fromleft: false
            }

            /* Health bar with label representing number of remaining people to save */
            HealthBar
            {
                range: 7
                lost: game.saved > range ? range : game.saved
                label: (range - lost) + " people to save"
                fromleft: true
            }
        }
    }

    /* Row consisting of one column filled with turn information */
    Row
    {
        anchors.left: parent.left
        anchors.leftMargin: parent.width / 1.22

        anchors.top: parent.top
        anchors.topMargin: parent.height / 3

        /* Column consisting of turn information */
        Column
        {
            spacing: game.height / 40

            /* Text displaying actual player */
            Text
            {
                font.bold: true
                font.pointSize: (game.height + game.width) / 120
                color: Theme.button

                text: "PLAYER " + game.onMove
            }

            /* Text displaying actual turn */
            Text
            {
                font.bold: true
                font.pointSize: (game.height + game.width) / 140
                color: Theme.button

                text: "Turn: " + game.turn
            }

            /* Text displaying number of remaining actions for actual player */
            Text
            {
                font.bold: true
                font.pointSize: (game.height + game.width) / 140
                color: game.currentLoad ? Theme.combo_box : Theme.button

                text: game.currentAP + " move(s) left"
            }

            /* Text displaying load carried by actual player */
            Text
            {
                font.bold: true
                font.pointSize: (game.height + game.width) / 140
                color: Theme.button

                text: game.currentLoad ? "Carried: " : "Carried: 0"

                /* Rectangle representing carried load */
                Rectangle
                {
                    anchors.left: parent.right
                    visible: game.currentLoad ? true : false

                    width: gameboard.width / 38
                    height: gameboard.height / 30

                    /* Image representing carried load */
                    Image
                    {
                        anchors.fill: parent

                        source: "../img/gamesquare_realalert.jpg"

                        /* Rectangle representing possibility of unloading carried load */
                        Rectangle
                        {
                            anchors.fill: parent

                            color: unloadable ? Theme.clickable_state : "transparent"
                            opacity: 0.6
                        }
                    }

                    /* Mouse area unloading carried load upon clicking */
                    MouseArea
                    {
                        enabled: unloadable ? true : false
                        anchors.fill: parent

                        onClicked: game.playerAction (-1, -1)
                    }
                }
            }
        }
    }

    /* Row consisting of one column filled with game buttons */
    Row
    {
        anchors.right: parent.right
        anchors.rightMargin: parent.width / 30

        anchors.bottom: parent.bottom
        anchors.bottomMargin: parent.height / 30

        /* Column consisting of game buttons */
        Column
        {
            spacing: game.height / 40

            /* Button for pausing and resuming game */
            GameButton
            {
                id: pause_button
                operation: "Pause Game"
            }

            /* Button for saving game */
            // TODO: add proper functionality
            GameButton
            {
                operation: "Save Game"
            }

            /* Button for exiting game screen */
            GameButton
            {
                operation: "Exit Game"
            }
        }
    }
}

//--------------------------------------------------------------------------------
// End of file GameScreen.qml
//--------------------------------------------------------------------------------
