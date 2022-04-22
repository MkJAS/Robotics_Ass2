%Close figures, clear command window and clear all variables every time the
%code is run.
clear all;
close all;
clc;

%Add logfile
logFile = log4matlab('13213537.log');

% Add ground image and set the size of the world
hold on;
worldCoords = 2.2;
axis([-worldCoords worldCoords -worldCoords worldCoords 0 worldCoords]); %minX maxX minY maxY minZ maxZ
surf([-worldCoords, -worldCoords; worldCoords, worldCoords], [-worldCoords, worldCoords; -worldCoords, worldCoords], [0, 0; 0, 0], 'CData', imread('marble.jpg'), 'FaceColor', 'texturemap');

% Adding objects to scene
tableHeight = 0.711547;
tableLength = 1.181 * 2;
tableWidth = 0.7387 * 2;
tablegapX = 0.005;
PlaceObject('Table.ply', [0, - (tablegapX + tableWidth / 2), 0]);
PlaceObject('Table.ply', [0, (tablegapX + tableWidth / 2), 0]);

%Set robot base locations
baseUR3 = [-0.4, -0.2, tableHeight];
baseUR5 = [0.7, -0.4, tableHeight];

%Add Safety Equipment
safetyEquipmentOffsetY = 1.34;
fenceOffsetX = 1.75;
smokeDetectorHeight = 1.8;
PlaceObject('FireExtinguisher.ply', [-1.3, safetyEquipmentOffsetY, 0]);
PlaceObject('FireExtinguisher.ply', [1.3, safetyEquipmentOffsetY, 0]);
PlaceObject('FireExtinguisher.ply', [1.3, -safetyEquipmentOffsetY, 0]);
PlaceObject('FireExtinguisher.ply', [-1.3, -safetyEquipmentOffsetY, 0]);
PlaceObject('EmergencyButton.ply', [-1.1, safetyEquipmentOffsetY, tableHeight]);
PlaceObject('EmergencyButton.ply', [-1.1, -safetyEquipmentOffsetY, tableHeight]);
PlaceObject('WiredFence.ply', [-fenceOffsetX, 0, 0]);
PlaceObject('WiredFence.ply', [fenceOffsetX, 0, 0]);
PlaceObject('SmokeDetectorFlipped.ply', [- (fenceOffsetX - 0.02), 0, smokeDetectorHeight]);
PlaceObject('SmokeDetector.ply', [(fenceOffsetX - 0.02), 0, smokeDetectorHeight]);

%% *Bricks Starting Locations
gridCoordX = 0;
gridCoordY = 0.05;

%row closest to UR3
brickPos1 = [gridCoordX - Brick.gapX, gridCoordY + Brick.gapY, tableHeight];
brickPos4 = [gridCoordX - Brick.gapX, gridCoordY, tableHeight];
brickPos3 = [gridCoordX - Brick.gapX, gridCoordY - Brick.gapY, tableHeight];

%middle row
brickPos7 = [gridCoordX, gridCoordY + Brick.gapY + 0.1, tableHeight];
brickPos9 = [gridCoordX, gridCoordY, tableHeight];
brickPos6 = [gridCoordX, gridCoordY - Brick.gapY, tableHeight];

%row closest to UR5
brickPos2 = [gridCoordX + Brick.gapX, gridCoordY + Brick.gapY, tableHeight];
brickPos8 = [gridCoordX + Brick.gapX, gridCoordY, tableHeight];
brickPos5 = [gridCoordX + Brick.gapX, gridCoordY - Brick.gapY, tableHeight];

%% Sort Bricks, give the UR3 the bricks closest to it
bricksGridContainer = [brickPos1; brickPos2; brickPos3; brickPos4; brickPos5; brickPos6; brickPos7; brickPos8; brickPos9];

for i = 1:9
    distances(i) = Distance(baseUR3, bricksGridContainer(i, :));
end

[~, index] = sort(distances);

for i = 1:4
    bricksGridUR3(i, :) = bricksGridContainer(index(i), :);
end

for i = 1:5
    bricksGridUR5(i, :) = bricksGridContainer(index(i + 4), :);
end

%% *Stacked Brick Locations
stackCoordX = 0;
stackCoordY = -0.5;

%row closest to UR3
brickUR3StackBot = [stackCoordX - Brick.length, stackCoordY, tableHeight];
brickUR3StackMid = [stackCoordX - Brick.length, stackCoordY, tableHeight + Brick.height];
brickUR3StackTop = [stackCoordX - Brick.length, stackCoordY, tableHeight + Brick.height * 2];

%middle row
brickMidStackBot = [stackCoordX, stackCoordY, tableHeight];
brickMidStackMid = [stackCoordX, stackCoordY, tableHeight + Brick.height];
brickMidStackTop = [stackCoordX, stackCoordY, tableHeight + Brick.height * 2];

%row closest to UR5
brickUR5StackBot = [stackCoordX + Brick.length, stackCoordY, tableHeight];
brickUR5StackMid = [stackCoordX + Brick.length, stackCoordY, tableHeight + Brick.height];
brickUR5StackTop = [stackCoordX + Brick.length, stackCoordY, tableHeight + Brick.height * 2];

%Order for the UR3 and UR5 to stack their bricks, important!!!
bricksStackedUR3 = [brickMidStackBot; brickUR3StackBot; brickUR3StackMid; brickUR3StackTop];
bricksStackedUR5 = [brickUR5StackBot; brickMidStackMid; brickUR5StackMid; brickUR5StackTop; brickMidStackTop];

%Create the bricks
for i = 1:4
    brickContainerUR3(i) = Brick(bricksGridUR3(i, :), bricksStackedUR3(i, :));
    brickContainerUR5(i) = Brick(bricksGridUR5(i, :), bricksStackedUR5(i, :));
end

brickContainerUR5(5) = Brick(bricksGridUR5(5, :), bricksStackedUR5(5, :));

%% Robots
%Set intermediary poses - found using teach
qIntermediaryUR3 = deg2rad([0 -90 -45 225 90 0]);
qIntermediaryUR5 = [-0.35 0 -pi / 2 -pi / 4 -pi / 4 pi / 2 0];

%Instantiate robots
robotUR3 = UR3(transl(baseUR3), qIntermediaryUR3);
robotUR5 = LinearUR5(transl(baseUR5) * trotz(-pi / 2), qIntermediaryUR5);

% robotUR3.model.teach();
%robotUR5.model.teach();

% %*Given an end effector pose, determine a joint state
% transformEndEffector = transl([0.2, -0.8, 1.2]) * trotx(pi) * troty(pi / 6);
% posePlot = trplot(transformEndEffector, 'length', 0.3);
% qDemo = robotUR5.model.ikcon(transformEndEffector);
% AnimateRobots(logFile, robotUR5, robotUR5.model.getpos(), qDemo);
% pause(6);
% try delete(posePlot); end

%Find current q to move robots to intermediary pose
qCurrentUR3 = robotUR3.model.getpos();
qCurrentUR5 = robotUR5.model.getpos();
AnimateRobots(logFile, robotUR5, qCurrentUR5, robotUR5.qIntermediary, robotUR3, qCurrentUR3, robotUR3.qIntermediary);

% %* Build the great wall of SafeCo
% for i = 1:4
%     PlaceBricks(logFile, robotUR5, brickContainerUR5(i), robotUR3, brickContainerUR3(i));
% end

% PlaceBricks(logFile, robotUR5, brickContainerUR5(5));
PlaceBrick(robotUR5, brickContainerUR5(5));

% %* Demonstrate Workspace
% pause(3);
% robotUR3.ShowReach();
% robotUR5.ShowReach();
