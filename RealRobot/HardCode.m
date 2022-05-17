%% Hardcoded Pick & Place code for Sensors & Control

% Any command with send in it has issues working from within a function

close all;
clear all;
clc;

rosshutdown; % Call this at the start just in case
rosinit; % Initialise connection

%% Initialise Dobot

[safetyStatePublisher, safetyStateMsg] = rospublisher('/dobot_magician/target_safety_status');
safetyStateMsg.Data = 2;
send(safetyStatePublisher, safetyStateMsg);

pause(25); % Long pause as robot needs to be fully initialised before starting

fprintf('\nDobot is initialised\n');

%% Figuring Out Values
x = 0.184;
z = -0.065;
zBasket = 0.09;
locationBlockGreen = [0.3 -0.033 z];
locationBlockYellow = [0.195 -0.169 z];
locationBlockOrange = [0.246 -0.08 z];
locationBasket = [0.287 0.093 zBasket];

pause(1)

%% Movement of the End Effector to Hover Over Object
target = locationBlockGreen
targetAbove = target;
targetAbove(3) = target(3) + 0.05;
% fpring(targetAbove);
pauseTime = 3;

[targetEndEffectorPub, targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
[targetEndEffectorPub, targetEndEffectorMsg] = MoveDobot(targetAbove)
send(targetEndEffectorPub, targetEndEffectorMsg); % Send command to move

pause(pauseTime)

[targetEndEffectorPub, targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
[targetEndEffectorPub, targetEndEffectorMsg] = MoveDobot(target)
send(targetEndEffectorPub, targetEndEffectorMsg); % Send command to move

state = 1; % Binary operator - change to toggle grip

[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
toolStateMsg.Data = [1 state];
send(toolStatePub, toolStateMsg); % Send command to gripper

pause (pauseTime)
target(3) = zBasket;

[targetEndEffectorPub, targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
[targetEndEffectorPub, targetEndEffectorMsg] = MoveDobot(target)
send(targetEndEffectorPub, targetEndEffectorMsg); % Send command to move

pause(pauseTime)

target = locationBasket

[targetEndEffectorPub, targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
[targetEndEffectorPub, targetEndEffectorMsg] = MoveDobot(target)
send(targetEndEffectorPub, targetEndEffectorMsg); % Send command to move

pause(pauseTime)
state = 0; % Binary operator - change to toggle grip

[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
toolStateMsg.Data = [1 state];
send(toolStatePub, toolStateMsg); % Send command to gripper

%%
%% Movement of the End Effector to Hover Over Object
target = locationBlockOrange;
targetAbove = target;
targetAbove(3) = target(3) + 0.05;
% fpring(targetAbove);

[targetEndEffectorPub, targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
[targetEndEffectorPub, targetEndEffectorMsg] = MoveDobot(targetAbove)
send(targetEndEffectorPub, targetEndEffectorMsg); % Send command to move

pause(pauseTime)

[targetEndEffectorPub, targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
[targetEndEffectorPub, targetEndEffectorMsg] = MoveDobot(target)
send(targetEndEffectorPub, targetEndEffectorMsg); % Send command to move

state = 1; % Binary operator - change to toggle grip

[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
toolStateMsg.Data = [1 state];
send(toolStatePub, toolStateMsg); % Send command to gripper

pause (pauseTime)
target(3) = zBasket;

[targetEndEffectorPub, targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
[targetEndEffectorPub, targetEndEffectorMsg] = MoveDobot(target)
send(targetEndEffectorPub, targetEndEffectorMsg); % Send command to move

pause(pauseTime)

target = locationBasket

[targetEndEffectorPub, targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
[targetEndEffectorPub, targetEndEffectorMsg] = MoveDobot(target)
send(targetEndEffectorPub, targetEndEffectorMsg); % Send command to move

pause(pauseTime)
state = 0; % Binary operator - change to toggle grip

[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
toolStateMsg.Data = [1 state];
send(toolStatePub, toolStateMsg); % Send command to gripper
%%
%% Movement of the End Effector to Hover Over Object
target = locationBlockYellow
targetAbove = target;
targetAbove(3) = target(3) + 0.05;
% fpring(targetAbove);

[targetEndEffectorPub, targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
[targetEndEffectorPub, targetEndEffectorMsg] = MoveDobot(targetAbove)
send(targetEndEffectorPub, targetEndEffectorMsg); % Send command to move

pause(pauseTime)

[targetEndEffectorPub, targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
[targetEndEffectorPub, targetEndEffectorMsg] = MoveDobot(target)
send(targetEndEffectorPub, targetEndEffectorMsg); % Send command to move

state = 1; % Binary operator - change to toggle grip

[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
toolStateMsg.Data = [1 state];
send(toolStatePub, toolStateMsg); % Send command to gripper

pause (pauseTime)
target(3) = zBasket;

[targetEndEffectorPub, targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
[targetEndEffectorPub, targetEndEffectorMsg] = MoveDobot(target)
send(targetEndEffectorPub, targetEndEffectorMsg); % Send command to move

pause(pauseTime)

target = locationBasket
% target(3) =  target(3) + 0.05;

[targetEndEffectorPub, targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
[targetEndEffectorPub, targetEndEffectorMsg] = MoveDobot(target)
send(targetEndEffectorPub, targetEndEffectorMsg); % Send command to move

pause(pauseTime)
state = 0; % Binary operator - change to toggle grip

[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
toolStateMsg.Data = [1 state];
send(toolStatePub, toolStateMsg); % Send command to gripper
