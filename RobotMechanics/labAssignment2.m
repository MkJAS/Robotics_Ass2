%Close figures, clear command window and clear all variables every time the
%code is run.
clc;
clear all;
close all;

%Add logfile
logFile = log4matlab('assignment2.log');

% Add ground image and set the size of the world
hold on;
worldCoords = 0.4;
axis([-worldCoords worldCoords -worldCoords worldCoords 0.7 1.3]); %minX maxX minY maxY minZ maxZ
surf([-worldCoords, -worldCoords; worldCoords, worldCoords], [-worldCoords, worldCoords; -worldCoords, worldCoords], [0, 0; 0, 0], 'CData', imread('marble.jpg'), 'FaceColor', 'texturemap');

% Adding objects to scene
tableHeight = 0.711547;
tableLength = 1.181 * 2;
tableWidth = 0.7387 * 2;
tablegapX = 0.005;
PlaceObject('Table.ply', [0, 0, 0]);

%Set robot base locations
baseDobot = [0, 0, tableHeight];

%Add Safety Equipment
PlaceObject('EmergencyButton.ply', [0.5, 0.5, tableHeight]);

% Plot lines between lightcurtains
% LightCurtain(tableHeight);

%% Objects
PlaceObject('Basket.ply', [0.25, 0.025, tableHeight]);

%% Robots
robotDobot = Dobot(transl(baseDobot));

% *Find current q to move robots to intermediary pose
AnimateRobots(logFile, robotDobot, robotDobot.qIntermediary);
RotateRobot(logFile, robotDobot, 0);

%%* Simulation movements
% %!strawberry
% locationStrawberry = [0.22, 0, tableHeight];
% strawberry = Strawberry(locationStrawberry);
% PickupObject(robotDobot, strawberry);
% RotateRobot(logFile, robotDobot, -90);
% PositionObject(robotDobot, [0, -0.2, tableHeight], 'strawberry');

% %!grape
% locationGrape = [0.05, -0.15, tableHeight];
% grape = Grape(locationGrape);
% PickupObject(robotDobot, grape);
% RotateRobot(logFile, robotDobot, 0);
% PositionObject(robotDobot, locationStrawberry, 'grape');

%!pill
locationPill = [0.05, -0.15, tableHeight];
pill = Pill(locationPill);
PickupObject(robotDobot, pill);
RotateRobot(logFile, robotDobot, -90);
PositionObject(robotDobot, [0, -0.2, tableHeight], 'pill');

%!lego
locationLego = [0.22, 0, tableHeight];
lego = Grape(locationLego);
PickupObject(robotDobot, lego);
RotateRobot(logFile, robotDobot, 0);
PositionObject(robotDobot, locationStrawberry, 'lego');

% RMRC(locationStrawberry, 1, robotDobot);
% robotDobot.model.teach();
