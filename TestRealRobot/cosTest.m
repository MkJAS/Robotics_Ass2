%% Hardcoded Pick & Place code for Sensors & Control

% Any command with send in it has issues working from within a function

close all;
clear all;
clc;

rosshutdown;        % Call this at the start just in case
rosinit;            % Initialise connection

%% Initialise Dobot

[safetyStatePublisher,safetyStateMsg] = rospublisher('/dobot_magician/target_safety_status');
safetyStateMsg.Data = 2;
send(safetyStatePublisher,safetyStateMsg);

pause(25);          % Long pause as robot needs to be fully initialised before starting

fprintf('\nDobot is initialised\n');

%% Figuring Out Values

x_cam = 41.2;     % x coordinate of cam (y coordinate of robot)
y_cam = 66.3;       % y coordinate of cam (x coordinate of robot)

y_offset = 0.0075;   % From y = 0 on the global coordinate frame

x = 0.2665 -(y_cam/1000);
y = -(x_cam/1000) + y_offset;

offset = 0.004;
theta = atan(y/x);
Tx = offset*cos(theta);
Ty = offset*sin(theta);

x_robot = x - Tx
y_robot = y + Ty

%% Figuring Out Values

x_cam2 = -156.2;     % x coordinate of cam (y coordinate of robot)
y_cam2 = 60.3;       % y coordinate of cam (x coordinate of robot)

y_offset2 = 0.0075;   % From y = 0 on the global coordinate frame

x2 = 0.2665 -(y_cam2/1000);
y2 = -(x_cam2/1000) + y_offset2;

offset2 = 0.004;
theta2 = atan(y/x);
Tx2 = offset2*cos(theta2);
Ty2 = offset2*sin(theta2);

x_robot2 = x2 - Tx2
y_robot2 = y2 - Ty2

pause(1)

%% Movement of the End Effector to Hover Over Object

fprintf('Dobot is moving to\n');              % Display end effector target position
target = [x_robot,y_robot,0]

[targetEndEffectorPub,targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
    
targetEndEffectorMsg.Position.X = target(1);
targetEndEffectorMsg.Position.Y = target(2);
targetEndEffectorMsg.Position.Z = target(3);
    
target_rotation = [0,0,0];
qua = eul2quat(target_rotation);
targetEndEffectorMsg.Orientation.W = qua(1);
targetEndEffectorMsg.Orientation.X = qua(2);
targetEndEffectorMsg.Orientation.Y = qua(3);
targetEndEffectorMsg.Orientation.Z = qua(4);
     
send(targetEndEffectorPub,targetEndEffectorMsg);         % Send command to move

pause(5)

fprintf('Dobot has completed translation\n');            % Display end effector current position

%% Movement of the End Effector to Desired Object

fprintf('Dobot is moving to\n');              % Display end effector target position
target = [x_robot,y_robot,-0.03]

[targetEndEffectorPub,targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
    
targetEndEffectorMsg.Position.X = target(1);
targetEndEffectorMsg.Position.Y = target(2);
targetEndEffectorMsg.Position.Z = target(3);
    
target_rotation = [0,0,0];
qua = eul2quat(target_rotation);
targetEndEffectorMsg.Orientation.W = qua(1);
targetEndEffectorMsg.Orientation.X = qua(2);
targetEndEffectorMsg.Orientation.Y = qua(3);
targetEndEffectorMsg.Orientation.Z = qua(4);
     
send(targetEndEffectorPub,targetEndEffectorMsg);         % Send command to move

pause(2)

fprintf('Dobot has completed translation\n');            % Display end effector current position

%% Pick Up Object

state = 1;                        % Binary operator - change to toggle grip

[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
toolStateMsg.Data = [1 state];
send(toolStatePub,toolStateMsg);  % Send command to gripper

pause (2)

fprintf('\nObject has been picked up\n');            % Display message for user

%% Movement of the End Effector to Hover Over Object

fprintf('Dobot is moving to\n');              % Display end effector target position
target = [x_robot,y_robot,0.03]

[targetEndEffectorPub,targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
    
targetEndEffectorMsg.Position.X = target(1);
targetEndEffectorMsg.Position.Y = target(2);
targetEndEffectorMsg.Position.Z = target(3);
    
target_rotation = [0,0,0];
qua = eul2quat(target_rotation);
targetEndEffectorMsg.Orientation.W = qua(1);
targetEndEffectorMsg.Orientation.X = qua(2);
targetEndEffectorMsg.Orientation.Y = qua(3);
targetEndEffectorMsg.Orientation.Z = qua(4);
     
send(targetEndEffectorPub,targetEndEffectorMsg);         % Send command to move

pause(2)

fprintf('Dobot has completed translation\n');            % Display end effector current position

%% Movement of the End Effector to Hover Over Desired Space

fprintf('Dobot is moving to\n');              % Display end effector target position
target = [x_robot2,y_robot2,0.03]

[targetEndEffectorPub,targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
    
targetEndEffectorMsg.Position.X = target(1);
targetEndEffectorMsg.Position.Y = target(2);
targetEndEffectorMsg.Position.Z = target(3);
    
target_rotation = [0,0,0];
qua = eul2quat(target_rotation);
targetEndEffectorMsg.Orientation.W = qua(1);
targetEndEffectorMsg.Orientation.X = qua(2);
targetEndEffectorMsg.Orientation.Y = qua(3);
targetEndEffectorMsg.Orientation.Z = qua(4);
     
send(targetEndEffectorPub,targetEndEffectorMsg);         % Send command to move

pause(5)

fprintf('Dobot has completed translation\n');            % Display end effector current position

%% Movement of the End Effector to Desired Space

fprintf('Dobot is moving to\n');              % Display end effector target position
target = [x_robot2,y_robot2,0.0125]

[targetEndEffectorPub,targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
    
targetEndEffectorMsg.Position.X = target(1);
targetEndEffectorMsg.Position.Y = target(2);
targetEndEffectorMsg.Position.Z = target(3);
    
target_rotation = [0,0,0];
qua = eul2quat(target_rotation);
targetEndEffectorMsg.Orientation.W = qua(1);
targetEndEffectorMsg.Orientation.X = qua(2);
targetEndEffectorMsg.Orientation.Y = qua(3);
targetEndEffectorMsg.Orientation.Z = qua(4);
     
send(targetEndEffectorPub,targetEndEffectorMsg);         % Send command to move

pause(5)

fprintf('Dobot has completed translation\n');            % Display end effector current position

%% Release Object

state = 0;                        % Binary operator - change to toggle grip

[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
toolStateMsg.Data = [1 state];
send(toolStatePub,toolStateMsg);  % Send command to gripper

pause (2)

fprintf('\nObject has been picked up\n');            % Display message for user



