//================================================================================
// BACHELOR THESIS - AUGMENTED REALITY FOR DESK GAMES
//================================================================================

/*
 * Filename:       gameboard.js
 * Date:           27. 11. 2017
 * Subject:        IBP - Bachelor Thesis
 * Description:    Javascript file for gameboard
 * Author:         Juraj Bačovčin
 *                 (xbacov04@stud.fit.vutbr.cz)
 */

/* Numbers representing map objects defined by rules */
var maxMedics = 8;
var maxAlerts = 3;
var maxColumn = 10;
var maxRow = 8;
var maxPlayers = 6;
var maxIndex = maxRow * maxColumn;
var addedActionPoints = 4;

/* Supporting variables */
var actualAlerts;
var component;

/* Arrays representing board pieces, players and medics */
var board = new Array (maxIndex);
var players = new Array (maxPlayers);
var medics = new Array (maxMedics);

/* Function returning array index based on column and row of game object */
function index (column, row)
{
    return column + (row * maxColumn);
}

/* Function initializing gameboard and starting game */
function startGame ()
{
    for (var i = 0; i < maxIndex; i++) if (board [i] != null) board [i].destroy ();

    for (var column = 0; column < maxColumn; column++)
    {
        for (var row = 0; row < maxRow; row++)
        {
            board [index (column, row)] = null;
            createSquare (column, row);
        }
    }
    createWalls ();
    createMedics ();

    for (var counter = 0; counter < game.players; counter++)
    {
        players [counter] = null;
        createPlayer (counter);
    }

    for (actualAlerts = 0; actualAlerts < maxAlerts; actualAlerts++) addAlert ();
    for (var fires = 0; fires < (game.difficulty * 5); fires++) addFire ();

    firstMove ();
}

/* Function used for creating board piece */
function createSquare (column, row)
{
    if (component == null) component = Qt.createComponent ("../qml/GameSquare.qml");

    if (component.status == Component.Ready)
    {
        var dynamicObject = component.createObject (gameboard);

        if (dynamicObject == null)
        {
            showErrorMessage ("Error creating gameboard.")
            return false;
        }

        dynamicObject.column = column;
        dynamicObject.row = row;

        board [index (column, row)] = dynamicObject;
    }

    else
    {
        showErrorMessage ("Error loading square component.")
        return false;
    }

    return true;
}

/* Function used for creating players */
function createPlayer (counter)
{
    component = Qt.createComponent ("../qml/Player.qml");

    if (component.status == Component.Ready)
    {
        var dynamicObject = component.createObject (gameboard);

        if (dynamicObject == null)
        {
            showErrorMessage ("Error creating player.")
            return false;
        }

        players [counter] = dynamicObject;
    }

    else
    {
        showErrorMessage ("Error loading player component.")
        return false;
    }

    return true;
}

/* Function used for creating walls */
// TODO: dynamically creating walls based on background picture or text file
function createWalls ()
{
    if (game.building == 0)
    {
        board [11].leftWall = "full";
        board [21].leftWall = "full";
        board [31].leftWall = "opened";
        board [41].leftWall = "full";
        board [51].leftWall = "full";
        board [61].leftWall = "full";

        board [33].leftWall = "closed";
        board [43].leftWall = "full";

        board [14].leftWall = "closed";
        board [24].leftWall = "full";

        board [16].leftWall = "full";
        board [26].leftWall = "closed";
        board [56].leftWall = "full";
        board [66].leftWall = "closed";

        board [37].leftWall = "full";
        board [47].leftWall = "closed";

        board [58].leftWall = "full";
        board [68].leftWall = "closed";

        board [19].leftWall = "full";
        board [29].leftWall = "full";
        board [39].leftWall = "full";
        board [49].leftWall = "opened";
        board [59].leftWall = "full";
        board [69].leftWall = "full";

        board [11].topWall = "full";
        board [12].topWall = "full";
        board [13].topWall = "full";
        board [14].topWall = "full";
        board [15].topWall = "full";
        board [16].topWall = "opened";
        board [17].topWall = "full";
        board [18].topWall = "full";

        board [33].topWall = "full";
        board [34].topWall = "full";
        board [35].topWall = "full";
        board [36].topWall = "full";
        board [37].topWall = "full";
        board [38].topWall = "closed";

        board [51].topWall = "full";
        board [52].topWall = "full";
        board [53].topWall = "full";
        board [54].topWall = "closed";
        board [55].topWall = "full";
        board [56].topWall = "full";
        board [57].topWall = "full";
        board [58].topWall = "full";

        board [71].topWall = "full";
        board [72].topWall = "full";
        board [73].topWall = "opened";
        board [74].topWall = "full";
        board [75].topWall = "full";
        board [76].topWall = "full";
        board [77].topWall = "full";
        board [78].topWall = "full";
    }

    else if (game.building == 1)
    {
        board [11].leftWall = "full";
        board [21].leftWall = "full";
        board [31].leftWall = "opened";
        board [41].leftWall = "full";
        board [51].leftWall = "full";
        board [61].leftWall = "full";

        board [14].leftWall = "full";
        board [24].leftWall = "full";
        board [34].leftWall = "full";
        board [54].leftWall = "full";
        board [64].leftWall = "full";

        board [16].leftWall = "closed";
        board [26].leftWall = "full";
        board [36].leftWall = "closed";

        board [47].leftWall = "closed";
        board [57].leftWall = "full";
        board [67].leftWall = "full";

        board [19].leftWall = "full";
        board [29].leftWall = "full";
        board [39].leftWall = "full";
        board [49].leftWall = "full";
        board [59].leftWall = "full";
        board [69].leftWall = "full";

        board [11].topWall = "full";
        board [12].topWall = "full";
        board [13].topWall = "full";
        board [14].topWall = "full";
        board [15].topWall = "full";
        board [16].topWall = "full";
        board [17].topWall = "full";
        board [18].topWall = "full";

        board [24].topWall = "full";
        board [25].topWall = "full";

        board [31].topWall = "full";
        board [32].topWall = "full";

        board [44].topWall = "full";
        board [45].topWall = "closed";
        board [46].topWall = "closed";
        board [47].topWall = "full";
        board [48].topWall = "full";

        board [51].topWall = "full";
        board [52].topWall = "full";
        board [53].topWall = "closed";
        board [54].topWall = "full";
        board [55].topWall = "full";
        board [56].topWall = "closed";

        board [71].topWall = "full";
        board [72].topWall = "full";
        board [73].topWall = "opened";
        board [74].topWall = "full";
        board [75].topWall = "full";
        board [76].topWall = "full";
        board [77].topWall = "full";
        board [78].topWall = "full";
    }

    else return;
}

/* Function used for creating medics */
// TODO: dynamically creating medics based on background picture or text file
function createMedics ()
{
    for (var i = 0; i < maxMedics; i++) medics [i] = {};

    medics [0].x = 5;
    medics [0].y = 0;

    medics [1].x = 6;
    medics [1].y = 0;

    medics [2].x = 3;
    medics [2].y = maxRow - 1;

    medics [3].x = 4;
    medics [3].y = maxRow - 1;

    medics [4].x = 0;
    medics [4].y = 3;

    medics [5].x = 0;
    medics [5].y = 4;

    medics [6].x = maxColumn - 1;
    medics [6].y = 3;

    medics [7].x = maxColumn - 1;
    medics [7].y = 4;
}

/* Function adding set number of action points to each player */
function addActionPoints ()
{
    for (var playerNumber = 0; playerNumber < game.players; playerNumber++)
    {
        for (var actionPointNumber = 0; actionPointNumber < addedActionPoints; actionPointNumber++)
        {
            players [playerNumber].actionPoints++;
        }
    }
}

/* Function disabling interactions with every gameboard component */
function disableAllSquares ()
{
    for (var i = 0; i < maxIndex; i++)
    {
        board [i].enabled = false;
        board [i].leftDoorsEnabled = false;
        board [i].topDoorsEnabled = false;
        board [i].actualPlayer = false;
        board [i].extinguishableFire = false;
    }
}

/* Function used for changing actual player */
function switchPlayer ()
{
    game.onMove++;
    game.currentAP = players [game.onMove - 1].actionPoints;
    game.currentLoad = players [game.onMove - 1].currentLoad;

    if (board [index (players [onMove - 1].column, players [onMove - 1].row)].state === "nothing") game.unloadable = true;

    else game.unloadable = false;

    if (turn > 0)
    {
        addSmoke ();
        checkAfterEffects ();
    }
}

/* Function used for measuring distance between specific player and specific medic */
function getDistance (playerX, playerY, medicX, medicY)
{
    if (board [index (medicX, medicY)].state === "fire" /*|| board [index (medicX, medicY)].player*/) return 100;

    else
    {
        var distanceX = Math.abs (playerX - medicX);
        var distanceY = Math.abs (playerY - medicY);
        var distance = distanceX + distanceY;

        return distance;
    }
}

/* Function used for finding medic closest to specific player */
// TODO: add possibility of choice in case two medics are equally close to specific player
function findClosestMedicTo (column, row)
{
    var lowestDistance = 100;
    var lowestMedic = 0;

    for (var i = 0; i < maxMedics; i++)
    {
        medics [i].distance = getDistance (column, row, medics [i].x, medics [i].y);

        if (medics [i].distance < lowestDistance)
        {
            lowestDistance = medics [i].distance;
            lowestMedic = i;
        }
    }

    if (lowestDistance === 100) game.gameOver ();

    return medics [lowestMedic];
}

/* Function moving player injured by fire to closest medic */
function hurtPlayer (column, row)
{
    var closestMedic = findClosestMedicTo (column, row);

    for (var actualPlayer = 0; actualPlayer < game.players; actualPlayer++)
    {
        if (players [actualPlayer].column === column && players [actualPlayer].row === row)
        {
            players [actualPlayer].column = closestMedic.x;
            players [actualPlayer].row = closestMedic.y;
            players [actualPlayer].overlapping = false;
            players [actualPlayer].currentLoad = false;

            board [index (players [actualPlayer].column, players [actualPlayer].row)].player = true;

            if (actualPlayer === (game.onMove - 1)) enableAvailableSquares ();;
        }
    }

    game.currentLoad = players [game.onMove - 1].currentLoad;
}

/* Function used for ending actual turn */
function finishTurn ()
{
    addActionPoints ();

    game.onMove = 0;

    switchPlayer ();

    game.turn++;
}

/* Function used for setting up starting position of each player */
function firstMove ()
{
    board [0].actualPlayer = true;

    for (var column = 0; column < maxColumn; column++)
    {
        /*if (! board [index (column, 0)].player)*/ board [index (column, 0)].enabled = true; // no need for condition
        /*if (! board [index (column, maxRow - 1)].player)*/ board [index (column, maxRow - 1)].enabled = true; // no need for condition
    }

    for (var row = 0; row < maxRow; row++)
    {
        /*if (! board [index (0, row)].player)*/ board [index (0, row)].enabled = true; // no need for condition
        /*if (! board [index (maxColumn - 1, row)].player)*/ board [index (maxColumn - 1, row)].enabled = true; // no need for condition
    }
}

/* Function enabling interactions with specific gameboard components based on their current states and rules */
function findAvailableSquares ()
{
    var playerX = players [onMove - 1].column;
    var playerY = players [onMove - 1].row;

    board [index (playerX, playerY)].enabled = true;
    board [index (playerX, playerY)].actualPlayer = true;
    board [index (playerX, playerY)].leftDoorsEnabled = true;
    board [index (playerX, playerY)].topDoorsEnabled = true;

    if (playerX > 0 /*&& ! board [index (playerX - 1, playerY)].player*/) // no need for composite condition
    {
        board [index (playerX - 1, playerY)].unpassable = false;

        if (! isNeighbor (playerX - 1, playerY, playerX, playerY, true, "nothing") &&
            ! isNeighbor (playerX - 1, playerY, playerX, playerY, true, "smoke") &&
            ! isNeighbor (playerX - 1, playerY, playerX, playerY, true, "questionMark") &&
            ! isNeighbor (playerX - 1, playerY, playerX, playerY, true, "realAlert")) board [index (playerX - 1, playerY)].unpassable = true;

        if (isNeighbor (playerX - 1, playerY, playerX, playerY, true, "fire")) board [index (playerX - 1, playerY)].extinguishableFire = true;

        board [index (playerX - 1, playerY)].enabled = true;
    }

    if (playerX < (maxColumn - 1) /*&& ! board [index (playerX + 1, playerY)].player*/) // no need for composite condition
    {
        board [index (playerX + 1, playerY)].unpassable = false;

        if (! isNeighbor (playerX + 1, playerY, playerX + 1, playerY, true, "nothing") &&
            ! isNeighbor (playerX + 1, playerY, playerX + 1, playerY, true, "smoke") &&
            ! isNeighbor (playerX + 1, playerY, playerX + 1, playerY, true, "questionMark") &&
            ! isNeighbor (playerX + 1, playerY, playerX + 1, playerY, true, "realAlert")) board [index (playerX + 1, playerY)].unpassable = true;

        if (isNeighbor (playerX + 1, playerY, playerX + 1, playerY, true, "fire")) board [index (playerX + 1, playerY)].extinguishableFire = true;

        board [index (playerX + 1, playerY)].enabled = true;
        board [index (playerX + 1, playerY)].leftDoorsEnabled = true;
    }

    if (playerY > 0 /*&& ! board [index (playerX, playerY - 1)].player*/) // no need for composite condition
    {
        board [index (playerX, playerY - 1)].unpassable = false;

        if (! isNeighbor (playerX, playerY - 1, playerX, playerY, false, "nothing") &&
            ! isNeighbor (playerX, playerY - 1, playerX, playerY, false, "smoke") &&
            ! isNeighbor (playerX, playerY - 1, playerX, playerY, false, "questionMark") &&
            ! isNeighbor (playerX, playerY - 1, playerX, playerY, false, "realAlert")) board [index (playerX, playerY - 1)].unpassable = true;

        if (isNeighbor (playerX, playerY - 1, playerX, playerY, false, "fire")) board [index (playerX, playerY - 1)].extinguishableFire = true;

        board [index (playerX, playerY - 1)].enabled = true;
    }

    if (playerY < (maxRow - 1) /*&& ! board [index (playerX, playerY + 1)].player*/) // no need for composite condition
    {
        board [index (playerX, playerY + 1)].unpassable = false;

        if (! isNeighbor (playerX, playerY + 1, playerX, playerY + 1, false, "nothing") &&
            ! isNeighbor (playerX, playerY + 1, playerX, playerY + 1, false, "smoke") &&
            ! isNeighbor (playerX, playerY + 1, playerX, playerY + 1, false, "questionMark") &&
            ! isNeighbor (playerX, playerY + 1, playerX, playerY + 1, false, "realAlert")) board [index (playerX, playerY + 1)].unpassable = true;

        if (isNeighbor (playerX, playerY + 1, playerX, playerY + 1, false, "fire")) board [index (playerX, playerY + 1)].extinguishableFire = true;

        board [index (playerX, playerY + 1)].enabled = true;
        board [index (playerX, playerY + 1)].topDoorsEnabled = true;
    }
}

/* Function refreshing enabled interactions with gameboard components */
function enableAvailableSquares ()
{
    disableAllSquares ();

    if (players [game.onMove - 1].ready) findAvailableSquares ();

    else firstMove ();
}

/* Function used for ending player actions */
function finishAction ()
{
    players [game.onMove - 1].actionPoints--;

    game.currentAP = players [game.onMove - 1].actionPoints;

    if (players [game.onMove - 1].currentLoad)
    {
        if (players [game.onMove - 1].column === 0 || players [game.onMove - 1].column === (maxColumn - 1) ||
            players [game.onMove - 1].row === 0 || players [game.onMove - 1].row === (maxRow - 1))
        {
            players [game.onMove - 1].currentLoad = false;
            board [index (players [game.onMove - 1].column, players [game.onMove - 1].row)].alert = false;

            game.currentLoad = players [game.onMove - 1].currentLoad;
            actualAlerts--;
            game.saved++;
        }
    }

    if (players [game.onMove - 1].actionPoints === 0)
    {
        if (game.onMove >= game.players) finishTurn ();

        else switchPlayer ();
    }

    enableAvailableSquares ();

    if (game.saved >= 7) game.gameOver (true);
}

/* Function used for moving actual player and properly handling related actions */
function moveCurrentPlayer (column, row)
{
    if (players [game.onMove - 1].ready && players [game.onMove - 1].column === column && players [game.onMove - 1].row === row)
    {
        if (game.onMove >= game.players) finishTurn ();

        else switchPlayer ();
    }

    else
    {        
        if (players [game.onMove - 1].currentLoad)
        {
            if (players [game.onMove - 1].actionPoints === 1)
            {
                showErrorMessage ("2 moves are needed to move while carrying someone.");
                return;
            }

            players [game.onMove - 1].actionPoints--;
            board [index (players [game.onMove - 1].column, players [game.onMove - 1].row)].alert = false;
            board [index (column, row)].alert = true;

            if (board [index (column, row)].state === "nothing") game.unloadable = true;

            else game.unloadable = false;
        }

        board [index (players [game.onMove - 1].column, players [game.onMove - 1].row)].player = false;

        players [game.onMove - 1].column = column;
        players [game.onMove - 1].row = row;
        players [game.onMove - 1].ready = true;

        board [index (column, row)].player = true;        

        if (board [index (column, row)].state != "nothing") players [game.onMove - 1].overlapping = true;

        else players [game.onMove - 1].overlapping = false;

        if (board [index (column, row)].state === "questionMark")
        {
            players [game.onMove - 1].overlapping = false;
            revealAlert (column, row);
        }

        finishAction ();
    }

    enableAvailableSquares ();
}

/* Function revealing board components represented by question marks */
function revealAlert (alertX, alertY)
{
    var alert = index (alertX, alertY);

    if (board [alert].alert) board [alert].state = "realAlert";

    else
    {
        board [alert].state = "nothing";
        actualAlerts--;
    }
}

/* Function used for executing specific action based on selected board component */
function executeAction (column, row)
{
    if (board [index (column, row)].state === "smoke" || board [index (column, row)].state === "fire") extinguishFire (column, row);

    else if (board [index (column, row)].state === "questionMark") revealAlert (column, row);

    else if (board [index (column, row)].state === "realAlert")
    {
        players [game.onMove - 1].actionPoints++;

        if (players [game.onMove - 1].currentLoad)
        {
            showErrorMessage ("Only 1 person can be carried at the same time.");
            return;
        }

        board [index (column, row)].state = "nothing";

        players [game.onMove - 1].currentLoad = true;
        players [game.onMove - 1].overlapping = false;

        game.currentLoad = players [game.onMove - 1].currentLoad;
        game.unloadable = true;
    }

    else showErrorMessage ("Unidentified player action.");
}

/* Function used for any player action other than movement */
function playerAction (column, row)
{
    if (players [game.onMove - 1].column < column)
    {
        var rightWall = board [index (column, row)].leftWall;

        if (rightWall === "full" || rightWall === "damaged") damageWall (column, row, true);

        else if (rightWall === "closed") board [index (column, row)].leftWall = "opened";

        else executeAction (column, row);
    }

    else if (players [game.onMove - 1].column > column)
    {
        var leftWall = board [index (column + 1, row)].leftWall;

        if (leftWall === "full" || leftWall === "damaged") damageWall (column + 1, row, true);

        else if (leftWall === "closed") board [index (column + 1, row)].leftWall = "opened";

        else executeAction (column, row);
    }

    else if (players [game.onMove - 1].row < row)
    {
        var bottomWall = board [index (column, row)].topWall;

        if (bottomWall === "full" || bottomWall === "damaged") damageWall (column, row, false);

        else if (bottomWall === "closed") board [index (column, row)].topWall = "opened";

        else executeAction (column, row);
    }

    else if (players [game.onMove - 1].row > row)
    {
        var topWall = board [index (column, row + 1)].topWall;

        if (topWall === "full" || topWall === "damaged") damageWall (column, row + 1, false);

        else if (topWall === "closed") board [index (column, row + 1)].topWall = "opened";

        else executeAction (column, row);
    }

    else
    {
        if (board [index (column, row)].state === "smoke" || board [index (column, row)].state === "fire") extinguishFire (column, row);

        else executeAction (column, row);
    }

    finishAction ();
}

/* Function used for laying carried person on board piece according to actual player's position */
function unloadAlert ()
{
    var alertX = players [game.onMove - 1].column;
    var alertY = players [game.onMove - 1].row;
    var alert = index (alertX, alertY);

    players [game.onMove - 1].overlapping = true;
    players [game.onMove - 1].currentLoad = false;
    game.currentLoad = players [game.onMove - 1].currentLoad;

    board [alert].state = "questionMark";
    board [alert].alert = true;

    revealAlert (alertX, alertY);
}

/* Function changing actual state of specific doors */
function moveDoors (column, row, isLeft)
{
    if (isLeft)
    {
        if (board [index (column, row)].leftWall === "closed") board [index (column, row)].leftWall = "opened";

        else if (board [index (column, row)].leftWall === "opened") board [index (column, row)].leftWall = "closed";

        else if (board [index (column, row)].leftWall != "destroyed") damageWall (column, row, isLeft);

        else return;
    }

    else
    {
        if (board [index (column, row)].topWall === "closed") board [index (column, row)].topWall = "opened";

        else if (board [index (column, row)].topWall === "opened") board [index (column, row)].topWall = "closed";

        else if (board [index (column, row)].topWall != "destroyed") damageWall (column, row, isLeft);

        else return;
    }

    finishAction ();
}

/* Function used to decide whether specific gameboard component is right next to other specific gameboard component */
function isNeighbor (x, y, wallX, wallY, isLeft, wantedState)
{
    var wall = index (wallX, wallY);

    if (board [index (x, y)].state === wantedState)
    {
        if (isLeft && board [wall].leftWall != "full" && board [wall].leftWall != "closed" && board [wall].leftWall != "damaged") return true;

        if (! isLeft && board [wall].topWall != "full" && board [wall].topWall != "closed" && board [wall].topWall != "damaged") return true;
    }

    return false;
}

/* Function changing actual state of specific wall */
function damageWall (wallX, wallY, isLeft)
{
    var wall = index (wallX, wallY);

    if (isLeft)
    {
        if (board [wall].leftWall === "full") board [wall].leftWall = "damaged";

        else if (board [wall].leftWall === "damaged" || board [wall].leftWall === "closed")
        {
            board [wall].leftWall = "destroyed";
            game.damaged++;
        }

        else showErrorMessage ("Unidentified damaging behavior.");
    }

    else
    {
        if (board [wall].topWall === "full") board [wall].topWall = "damaged";

        else if (board [wall].topWall === "damaged" || board [wall].topWall === "closed")
        {
            board [wall].topWall = "destroyed";
            game.damaged++;
        }

        else showErrorMessage ("Unidentified damaging behavior.");
    }
}

/* Function changing actual state of specific fire piece */
function extinguishFire (fireX, fireY)
{
    var fire = index (fireX, fireY);

    if (board [fire].state === "smoke")
    {
        if (board [fire].smokedAlert)
        {
            board [fire].smokedAlert = false;
            board [fire].state = "questionMark";

            if (players [game.onMove - 1].column === fireX && players [game.onMove - 1].row === fireY) revealAlert (fireX, fireY);
        }

        else
        {
            if (players [game.onMove - 1].column === fireX && players [game.onMove - 1].row === fireY) players [game.onMove - 1].overlapping = false;
            board [fire].state = "nothing";

            if (players [game.onMove - 1].currentLoad) game.unloadable = true;
        }
    }

    else board [fire].state = "smoke";
}

/* Function used for creating fire */
function createFire (fireX, fireY)
{
    var fire = index (fireX, fireY);

    if (board [fire].state === "questionMark" || board [fire].state === "realAlert" || board [fire].smokedAlert) actualAlerts--;

    board [fire].smokedAlert = false;
    board [fire].state = "fire";
}

/* Function used for creating explosion along with its aftermaths */
function createExplosion (fireX, fireY)
{
    var counter;

    counter = 0;
    while (true)
    {
        if (fireX > counter)
        {
            var leftWall = board [index (fireX - counter, fireY)].leftWall;

            if (leftWall === "full" || leftWall === "closed" || leftWall === "damaged")
            {
                damageWall (fireX - counter, fireY, true);
                break;
            }

            if (board [index (fireX - (counter + 1), fireY)].state != "fire")
            {
                createFire (fireX - (counter + 1), fireY);
                break;
            }

            counter++;
        }

        else break;
    }

    counter = 0;
    while (true)
    {
        if (fireX < (maxColumn - (1 + counter)))
        {
            var rightWall = board [index (fireX + (counter + 1), fireY)].leftWall;

            if (rightWall === "full" || rightWall === "closed" || rightWall === "damaged")
            {
                damageWall (fireX + (counter + 1), fireY, true);
                break;
            }

            if (board [index (fireX + (counter + 1), fireY)].state != "fire")
            {
                createFire (fireX + (counter + 1), fireY);
                break;
            }

            counter++;
        }

        else break;
    }

    counter = 0;
    while (true)
    {
        if (fireY > counter)
        {
            var topWall = board [index (fireX, fireY - counter)].topWall;

            if (topWall === "full" || topWall === "closed" || topWall === "damaged")
            {
                damageWall (fireX, fireY - counter, false);
                break;
            }

            if (board [index (fireX, fireY - (counter + 1))].state != "fire")
            {
                createFire (fireX, fireY - (counter + 1));
                break;
            }

            counter++;
        }

        else break;
    }

    counter = 0;
    while (true)
    {
        if (fireY < (maxRow - (1 + counter)))
        {
            var bottomWall = board [index (fireX, fireY + (counter + 1))].topWall;

            if (bottomWall === "full" || bottomWall === "closed" || bottomWall === "damaged")
            {
                damageWall (fireX, fireY + (counter + 1), false);
                break;
            }

            if (board [index (fireX, fireY + (counter + 1))].state != "fire")
            {
                createFire (fireX, fireY + (counter + 1));
                break;
            }

            counter++;
        }

        else break;
    }
}

/* Function creating smoke or fire based on surrounding fire */
function checkSmokeAround (smokeX, smokeY)
{
    board [index (smokeX, smokeY)].state = "smoke";

    if ((smokeX > 0 && isNeighbor (smokeX - 1, smokeY, smokeX, smokeY, true, "fire")) ||
        (smokeX < (maxColumn - 1) && isNeighbor (smokeX + 1, smokeY, smokeX + 1, smokeY, true, "fire")) ||
        (smokeY > 0 && isNeighbor (smokeX, smokeY - 1, smokeX, smokeY, false, "fire")) ||
        (smokeY < (maxRow - 1) && isNeighbor (smokeX, smokeY + 1, smokeX, smokeY + 1, false, "fire")))
        createFire (smokeX, smokeY);
}

/* Function creating fire along with its aftermaths based on surrounding fire */
function checkFireAround (fireX, fireY)
{
    if ((fireX > 0 && isNeighbor (fireX - 1, fireY, fireX, fireY, true, "fire")) ||
        (fireX < (maxColumn - 1) && isNeighbor (fireX + 1, fireY, fireX + 1, fireY, true, "fire")) ||
        (fireY > 0 && isNeighbor (fireX, fireY - 1, fireX, fireY, false, "fire")) ||
        (fireY < (maxRow - 1) && isNeighbor (fireX, fireY + 1, fireX, fireY + 1, false, "fire")))
    {
        createFire (fireX, fireY);
        checkAfterEffects ();
    }
}

/* Function actualizing gameboard and game states (along with possibility of game over) according to newly added smoke and fire */
function checkAfterEffects ()
{
    for (var column = 0; column < maxColumn; column++)
    {
        for (var row = 0; row < maxRow; row++)
        {
            if (board [index (column, row)].state === "smoke") checkFireAround (column, row);

            if (board [index (column, row)].state === "fire")
            {
                if (board [index (column, row)].alert)
                {
                    board [index (column, row)].alert = false;
                    game.dead++;
                }

                if (board [index (column, row)].player)
                {
                    board [index (column, row)].player = false;
                    hurtPlayer (column, row);
                }

                while (actualAlerts < maxAlerts)
                {
                    addAlert ();
                    actualAlerts++;
                }
            }
        }
    }

    for (var counter = 0; counter < game.players; counter++)
    {
        var boardState = board [index (players [counter].column, players [counter].row)].state;

        if (boardState === "nothing") players [counter].overlapping = false;

        else players [counter].overlapping = true;
    }

    if (game.dead >= 4 || game.damaged >= 24) game.gameOver (false);
}

/* Function used for adding smoke to gameboard according to rules */
function addSmoke ()
{
    var smokeX = Math.floor ((Math.random () * (maxColumn - 2)) + 1);
    var smokeY = Math.floor ((Math.random () * (maxRow - 2)) + 1);
    var smoke = index (smokeX, smokeY);

    if (board [smoke].state === "nothing") checkSmokeAround (smokeX, smokeY);

    else if (board [smoke].state === "smoke") createFire (smokeX, smokeY);

    else if (board [smoke].state === "fire") createExplosion (smokeX, smokeY);

    else if (board [smoke].state === "questionMark" || board [smoke].state === "realAlert")
    {
        board [smoke].smokedAlert = true;
        checkSmokeAround (smokeX, smokeY);
    }

    else showErrorMessage ("Unidentified smoking behavior.");
}

/* Function used for adding fire to gameboard according to rules (used only in the beginning) */
function addFire ()
{
    var fireX = Math.floor ((Math.random () * (maxColumn - 2)) + 1);
    var fireY = Math.floor ((Math.random () * (maxRow - 2)) + 1);
    var fire = index (fireX, fireY);

    if (board [fire].state != "nothing") addFire ();

    else board [fire].state = "fire";
}

/* Function used for adding alerts to gameboard according to rules */
function addAlert ()
{
    var real = Math.floor ((Math.random () * 2));

    var alertX = Math.floor ((Math.random () * (maxColumn - 2)) + 1);
    var alertY = Math.floor ((Math.random () * (maxRow - 2)) + 1);
    var alert = index (alertX, alertY);

    if (board [alert].state === "questionMark" || board [alert].smokedAlert) addAlert ();

    else
    {
        board [alert].state = "questionMark";

        if (real == 1) board [alert].alert = true;

        if (board [alert].player) revealAlert (alertX, alertY);
    }
}

//--------------------------------------------------------------------------------
// End of file gameboard.js
//--------------------------------------------------------------------------------
