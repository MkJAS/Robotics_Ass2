% sudo chmod 666 /dev/ttyUSB0;
% roslaunch dobot_magician_driver dobot_magician.launch

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

disp("Pausing for 40 seconds");
pause(40); % Long pause as robot needs to be fully initialised before starting

fprintf('\nDobot is initialised with the current parameters\n');

safetyStatusSubscriber = rossubscriber('/dobot_magician/safety_status');
pause(2); %Allow some time for MATLAB to start the subscriber
currentSafetyStatus = safetyStatusSubscriber.LatestMessage.Data
%% Movement of the End Effector to Hover Over Object
target = [0.32, 0.082, -0.0186];
   [targetEndEffectorPub, targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');

    targetEndEffectorMsg.Position.X = target(1);
    targetEndEffectorMsg.Position.Y = target(2);
    targetEndEffectorMsg.Position.Z = target(3);

    target_rotation = [0, 0, 0];
    qua = eul2quat(target_rotation);
    targetEndEffectorMsg.Orientation.W = qua(1);
    targetEndEffectorMsg.Orientation.X = qua(2);
    targetEndEffectorMsg.Orientation.Y = qua(3);
    targetEndEffectorMsg.Orientation.Z = qua(4);

% [targetEndEffectorPub, targetEndEffectorMsg] = MoveDobot(target);
send(targetEndEffectorPub, targetEndEffectorMsg);
fprintf('pause 10');
pause(10);
target = [0.32, 0.072, 0.0386];
   [targetEndEffectorPub, targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');

    targetEndEffectorMsg.Position.X = target(1);
    targetEndEffectorMsg.Position.Y = target(2);
    targetEndEffectorMsg.Position.Z = target(3);

    target_rotation = [0, 0, 0];
    qua = eul2quat(target_rotation);
    targetEndEffectorMsg.Orientation.W = qua(1);
    targetEndEffectorMsg.Orientation.X = qua(2);
    targetEndEffectorMsg.Orientation.Y = qua(3);
    targetEndEffectorMsg.Orientation.Z = qua(4);

% [targetEndEffectorPub, targetEndEffectorMsg] = MoveDobot(target);
send(targetEndEffectorPub, targetEndEffectorMsg);

target = [0.32, 0.082, -0.0186];
   [targetEndEffectorPub, targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');

    targetEndEffectorMsg.Position.X = target(1);
    targetEndEffectorMsg.Position.Y = target(2);
    targetEndEffectorMsg.Position.Z = target(3);

    target_rotation = [0, 0, 0];
    qua = eul2quat(target_rotation);
    targetEndEffectorMsg.Orientation.W = qua(1);
    targetEndEffectorMsg.Orientation.X = qua(2);
    targetEndEffectorMsg.Orientation.Y = qua(3);
    targetEndEffectorMsg.Orientation.Z = qua(4);

% [targetEndEffectorPub, targetEndEffectorMsg] = MoveDobot(target);
send(targetEndEffectorPub, targetEndEffectorMsg);
fprintf('pause 10');
pause(10);
target = [0.32, 0.072, 0.0386];
   [targetEndEffectorPub, targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');

    targetEndEffectorMsg.Position.X = target(1);
    targetEndEffectorMsg.Position.Y = target(2);
    targetEndEffectorMsg.Position.Z = target(3);

    target_rotation = [0, 0, 0];
    qua = eul2quat(target_rotation);
    targetEndEffectorMsg.Orientation.W = qua(1);
    targetEndEffectorMsg.Orientation.X = qua(2);
    targetEndEffectorMsg.Orientation.Y = qua(3);
    targetEndEffectorMsg.Orientation.Z = qua(4);

% [targetEndEffectorPub, targetEndEffectorMsg] = MoveDobot(target);
send(targetEndEffectorPub, targetEndEffectorMsg);

rosshutdown;


