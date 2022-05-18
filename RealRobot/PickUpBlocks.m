% Helpful Terminal Commands
% sudo chmod 666 / dev / ttyUSB0;
% roslaunch dobot_magician_driver dobot_magician.launch

close all;
clear all;
clc;

%% Initialise Dobot
rosshutdown;
rosinit;
[safetyStatePublisher, safetyStateMsg] = rospublisher('/dobot_magician/target_safety_status');
safetyStateMsg.Data = 2;
send(safetyStatePublisher, safetyStateMsg);
pause(24); % Long pause to wait for robot to properly initialise
fprintf('\nDobot is initialised\n');

%% Block Location Extraction
referenceImage = load('image.mat');
blockCoordinates = cam2Robot(referenceImage.image);

locationYellowBlock = [(blockCoordinates.yellow)];
locationGreenBlock = [(blockCoordinates.green)];
locationOrangeBlock = [(blockCoordinates.orange)];
locationBasket = [(blockCoordinates.basket)];

%% todo Pickup Green Block and drop into Basket
%Move EE to above the green block
target = locationGreenBlock;
targetAbove = target;
targetAbove(3) = target(3) + 0.05; % Move the EE 5cm higher than the robot to prevent collisons with blocks and the basket rim
[targetEndEffectorPub, targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
[targetEndEffectorPub, targetEndEffectorMsg] = MoveDobot(targetAbove)
send(targetEndEffectorPub, targetEndEffectorMsg);
pauseDuration = 3; %Seconds
pause(pauseDuration)

%Move EE down to the green block
[targetEndEffectorPub, targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
[targetEndEffectorPub, targetEndEffectorMsg] = MoveDobot(target)
send(targetEndEffectorPub, targetEndEffectorMsg);

%Grip the green block
state = 1; % 1 = grip with End Effector
[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
toolStateMsg.Data = [1 state];
send(toolStatePub, toolStateMsg);
pause (pauseDuration)

%Move to the height of the basket rim
target(3) = basketRimZ;
[targetEndEffectorPub, targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
[targetEndEffectorPub, targetEndEffectorMsg] = MoveDobot(target)
send(targetEndEffectorPub, targetEndEffectorMsg);
pause(pauseDuration)

%Move the EE to the basket
target = locationBasket
[targetEndEffectorPub, targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
[targetEndEffectorPub, targetEndEffectorMsg] = MoveDobot(target)
send(targetEndEffectorPub, targetEndEffectorMsg);
pause(pauseDuration)

%Release the gripper
state = 0; % 0 = release End Effector Grip
[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
toolStateMsg.Data = [1 state];
send(toolStatePub, toolStateMsg);

%% todo Pickup Orange Block and drop into Basket
%Move EE to above the orange block
target = locationOrangeBlock;
targetAbove = target;
targetAbove(3) = target(3) + 0.05; % Move the EE 5cm higher than the robot to prevent collisons with blocks and the basket rim
[targetEndEffectorPub, targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
[targetEndEffectorPub, targetEndEffectorMsg] = MoveDobot(targetAbove)
send(targetEndEffectorPub, targetEndEffectorMsg);
pauseDuration = 3; %Seconds
pause(pauseDuration)

%Move EE down to the orange block
[targetEndEffectorPub, targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
[targetEndEffectorPub, targetEndEffectorMsg] = MoveDobot(target)
send(targetEndEffectorPub, targetEndEffectorMsg);

%Grip the orange block
state = 1; % 1 = grip with End Effector
[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
toolStateMsg.Data = [1 state];
send(toolStatePub, toolStateMsg);
pause (pauseDuration)

%Move to the height of the basket rim
target(3) = basketRimZ;
[targetEndEffectorPub, targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
[targetEndEffectorPub, targetEndEffectorMsg] = MoveDobot(target)
send(targetEndEffectorPub, targetEndEffectorMsg);
pause(pauseDuration)

%Move the EE to the basket
target = locationBasket
[targetEndEffectorPub, targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
[targetEndEffectorPub, targetEndEffectorMsg] = MoveDobot(target)
send(targetEndEffectorPub, targetEndEffectorMsg);
pause(pauseDuration)

%Release the gripper
state = 0; % 0 = release End Effector Grip
[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
toolStateMsg.Data = [1 state];
send(toolStatePub, toolStateMsg);

%% todo Pickup Yellow Block and drop into Basket
%Move EE to above the Yellow block
target = locationYellowBlock;
targetAbove = target;
targetAbove(3) = target(3) + 0.05; % Move the EE 5cm higher than the robot to prevent collisons with blocks and the basket rim
[targetEndEffectorPub, targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
[targetEndEffectorPub, targetEndEffectorMsg] = MoveDobot(targetAbove)
send(targetEndEffectorPub, targetEndEffectorMsg);
pauseDuration = 3; %Seconds
pause(pauseDuration)

%Move EE down to the Yellow block
[targetEndEffectorPub, targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
[targetEndEffectorPub, targetEndEffectorMsg] = MoveDobot(target)
send(targetEndEffectorPub, targetEndEffectorMsg);

%Grip the Yellow block
state = 1; % 1 = grip with End Effector
[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
toolStateMsg.Data = [1 state];
send(toolStatePub, toolStateMsg);
pause (pauseDuration)

%Move to the height of the basket rim
target(3) = basketRimZ;
[targetEndEffectorPub, targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
[targetEndEffectorPub, targetEndEffectorMsg] = MoveDobot(target)
send(targetEndEffectorPub, targetEndEffectorMsg);
pause(pauseDuration)

%Move the EE to the basket
target = locationBasket
[targetEndEffectorPub, targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
[targetEndEffectorPub, targetEndEffectorMsg] = MoveDobot(target)
send(targetEndEffectorPub, targetEndEffectorMsg);
pause(pauseDuration)

%Release the gripper
state = 0; % 0 = release End Effector Grip
[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
toolStateMsg.Data = [1 state];
send(toolStatePub, toolStateMsg);
