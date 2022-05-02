%Close figures, clear command window and clear all variables every time the
%code is run.
%test
clc;
clear all;
close all;

%Add logfile
logFile = log4matlab('assignment2.log');

% Add ground image and set the size of the world
hold on;
worldCoords = 0.6;
axis([-worldCoords worldCoords -worldCoords worldCoords 0.6 worldCoords + 0.8]); %minX maxX minY maxY minZ maxZ
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
offsetLightCurtain = 0.35;
PlaceObject('EmergencyButton.ply', [0.5, 0.5, tableHeight]);
PlaceObject('LightCurtain.ply', [-offsetLightCurtain, offsetLightCurtain, tableHeight]);
PlaceObject('LightCurtain.ply', [offsetLightCurtain, offsetLightCurtain, tableHeight]);

PlaceObject('LightCurtain.ply', [-offsetLightCurtain, -offsetLightCurtain, tableHeight]);
PlaceObject('LightCurtain.ply', [offsetLightCurtain, -offsetLightCurtain, tableHeight]);

%% Plot lines between lightcurtains
% offsetVertical = 0.1;
% 
% for index = 1:4
%     startP = ([-offsetLightCurtain, offsetLightCurtain, tableHeight + 0.05 + offsetVertical]);
%     endP = ([offsetLightCurtain, offsetLightCurtain, tableHeight + 0.05 + offsetVertical]);
%     plot3([startP(1), endP(1)], [startP(2), endP(2)], [startP(3), endP(3)], 'r');
%     offsetVertical = offsetVertical + 0.1;
% end

LightCurtainLasers;

%% Objects
% PlaceObject('Strawberry.ply', [-0.2, -0.2, tableHeight]);
% PlaceObject('Grape.ply', [0.2, 0.2, tableHeight]);
PlaceObject('Basket.ply', [0.1, 0.2, tableHeight]);

locationGrape = [0, -0.15, tableHeight];
%grape = Grape(locationGrape);

locationStrawberry = [0.2, 0, tableHeight];
strawberry = Strawberry(locationStrawberry);

%% Robots
robotDobot = Dobot(transl(baseDobot));

% *Find current q to move robots to intermediary pose
qCurrentDobot = robotDobot.model.getpos();
qTarget = deg2rad([0 45 45 0 0]);
AnimateRobots(logFile, robotDobot, qCurrentDobot, qTarget);

placeMentLocation = [0.1, -0.2, tableHeight];
MoveFruit(logFile, robotDobot, strawberry, locationGrape);

%robotDobot.model.teach();
