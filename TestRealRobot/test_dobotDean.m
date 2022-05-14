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

disp("Pausing for 25 seconds");
pause(25); % Long pause as robot needs to be fully initialised before starting

fprintf('\nDobot is initialised with the current parameters\n');

defaultEndEffectorPosition = [0.2591, 0, -0.0086] % Default end effector position
groundLevel = -0.0419 % Z value of the table

%% Movement of the End Effector to Hover Over Object
send(MoveDobot)
