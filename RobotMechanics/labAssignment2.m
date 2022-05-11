%Close figures, clear command window and clear all variables every time the
%code is run.
clc;
clear all;
close all;

% Add ground image and set the size of the world
hold on;
worldCoords = 0.6;
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

% LightCurtain(tableHeight);

locationCamera = [0.35, 0, tableHeight + 0.475];
PlaceObject('Camera.ply', locationCamera);

%% Objects
PlaceObject('Basket.ply', [0.25, 0.025, tableHeight]);

locationStrawberry = [0.22, -0.2, tableHeight];
locationFinalStrawberry = [0.29, 0, tableHeight];
strawberry = Strawberry(locationStrawberry);

locationGrape = [0.05, -0.18, tableHeight];
locationFinalGrape = [0.26, 0, tableHeight];
grape = Grape(locationGrape);

locationLego = [0.10, -0.25, tableHeight];
locationFinalLego = [0.23, 0, tableHeight];
lego = Lego(locationLego);

locationPill = [0.17, -0.23, tableHeight];
locationFinalPill = [0.2, 0, tableHeight];
pill = Pill(locationPill);

%% Robots
robotDobot = Dobot(transl(baseDobot));

% % *Find current q to move robots to intermediary pose
angleRotation = -60;
AnimateRobots(robotDobot, robotDobot.qIntermediary);
% RotateRobot(robotDobot, angleRotation);

% % %%* Simulation movements
% % %!strawberry
% PickupObject(robotDobot, strawberry);
% RotateRobot(robotDobot, 0);
% PositionObject(robotDobot, locationFinalStrawberry, 'strawberry');

% % !grape
% RotateRobot(robotDobot, angleRotation);
% PickupObject(robotDobot, grape);
% RotateRobot(robotDobot, 0);
% PositionObject(robotDobot, locationFinalGrape, 'grape');

% % !lego
% RotateRobot(robotDobot, angleRotation);
% PickupObject(robotDobot, lego);
% RotateRobot(robotDobot, 0);
% PositionObject(robotDobot, locationFinalLego, 'lego');

% % !pill
% RotateRobot(robotDobot, angleRotation);
% PickupObject(robotDobot, pill);
% RotateRobot(robotDobot, 0);
% PositionObject(robotDobot, locationFinalPill, 'pill');

% RotateRobot(robotDobot, -15);
% locationArm = [0.55, -0.2, tableHeight + 0.27];
% PlaceObject('Arm.ply', locationArm);

% Camera Orbit
axis vis3d

for i = 1:500
    camorbit(1, 0, 'data', [0.05 0.05 1])
    drawnow
end

LightCurtain(tableHeight);

for i = 1:500
    camorbit(1, 0, 'data', [0.05 0.05 1])
    drawnow
end
