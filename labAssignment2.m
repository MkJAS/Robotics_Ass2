%Close figures, clear command window and clear all variables every time the
%code is run.
clear all;
close all;
clc;

%Add logfile
logFile = log4matlab('labAssignment2.log');

% Add ground image and set the size of the world
hold on;
worldCoords = 0.6;
axis([-worldCoords worldCoords -worldCoords worldCoords 0.6 worldCoords + 0.6]); %minX maxX minY maxY minZ maxZ
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

%% Objects
PlaceObject('Strawberry.ply', [-0.2, -0.2, tableHeight]);
PlaceObject('Sphere.ply', [0.2, 0.2, tableHeight]);

%% Robots
%Set intermediary poses - found using teach
%qIntermediaryDobot = deg2rad([0 -90 -45 225 90 0]);

%Instantiate robots
robotDobot = Dobot(transl(baseDobot));

robotDobot.model.teach();

%Find current q to move robots to intermediary pose
%qCurrentDobot = robotDobot.model.getpos();
%AnimateRobots(logFile, robotDobot, qCurrentDobot, robotDobot.qIntermediary);
