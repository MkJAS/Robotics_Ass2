%% GENERAL
close %closes current fig. Can specify fig close(fig) or close all
clear %removes variables from the current workspace
clc % clear command window
clf %clears all children of figure

%DISPLAYs msg in command window
disp('this is a message');
input('press enter to continue');
error('stops running');
warning('warns in command window');
sprintf('distance between EE position is %0.4f', distance) %Printing

%DELAY - x in seconds
pause(x)

%ANGLE CONVERSION
radians = deg2rad(degrees)
degrees = rad2deg(rad)

drawnow(); %update figures and callbacks

%distance for positions - mpower: element wise power
distance = sqrt(sum((position1 - position2).^2));

%MATRIX
zeros(3, 4) % creates a matrix of zeros with 3 rows and 4 columns
eye(7, 2) % creates an eigen matrix of diagonal 1's with 7 rows and 2 columns

%% WORKSPACE, axes etc

% Set the size of the workspace when drawing the robot
workspace = [-2 2 -2 2 -0.05 2]; % xmin xmax ymin ymax zmin zmax
axis([-5 5 -5 5 0 3]); %minX maxX minY maxY minZ maxZ

%% TRANSFORMS
trotx(angle), troty(angle), trotz(angle) %rotational transform around respective axis by angle

%TRANSL
T = TRANSL(X, Y, Z)
% is an SE(3) homogeneous transform (4x4) representing a pure translation of X, Y and Z.

%% ROBOTS
robot.plot(q, 'workspace', workspace, 'scale', scale); %plots the robot with a given set of joint angles q within the given workspace and scale
robot.teach; %can control the robot and specify joint angles with a gui up to the joint limits.
robot.base = troty(pi); % Rotate the base around the Y axis so the Z axis faces downways

% Link('Theta', ____,'d',____,'a',_____,'alpha',____,'offset',____,'qlim',[ ... ])
%For Puma560
L1 = Link('d', 0, 'a', 0, 'alpha', pi / 2, 'offset', 0)
L2 = Link('d', 0, 'a', 0.4318, 'alpha', 0, 'offset', 0)
L3 = Link('d', 0.15, 'a', 0.0203, 'alpha', -pi / 2, 'offset', 0)
L4 = Link('d', 0.4318, 'a', 0, 'alpha', pi / 2, 'offset', 0)
L5 = Link('d', 0, 'a', 0, 'alpha', -pi / 2, 'offset', 0)
L6 = Link('d', 0, 'a', 0, 'alpha', 0, 'offset', 0)

%Link the links together to construct a robot
myPuma = SerialLink([L1 L2 L3 L4 L5 L6], 'name', "deansPuma")

%Set these joint angles to zero
q = zeros(1, 6);

%Plot the robot with joint angles of q
myPuma.plot(q)

%Find joint angles q1 from the current Puma position
q1 = myPuma.getpos()

%FKINE - Find transformation based on joint angles
%takes the current joint state and returns a transform to get there
forwardTransformation = myPuma.fkine(q)

%IKINE - Find joint angles based on transformation
%takes a forward transform and gives the required joint angles (q)
qFromForwardTransformation = myPuma.ikine(forwardTransformation)

% Q = R.ikcon(T, qCurrent) are the joint coordinates (1xN) corresponding to the robot
% end-effector pose T (4x4) which is a homogenenous transform.

newQ = robot.ikine(transl(x, y, z), qInitialJointGuess, [1, 1, 0, 0, 0, 0])
% transl(x,y,z) = transformation matrix of desired position
% [1,1,0,0,0,0] mask - solve the new joint angles for x and y, z and rpy ie
% orientation doesn't matter in this example

%MOVE ROBOT IN STRAIGHT LINE
currentQ = robot.ikine(transl(-0.75, y, 0), lastQ, [1, 1, 0, 0, 0, 0]);
robot.plot(currentQ);

for y = -0.5:0.05:0.5
    newQ = robot.ikine(transl(-0.75, y, 0), currentQ, [1, 1, 0, 0, 0, 0]);
    robot.animate(newQ);
end

%% Funcs to do
rpy2tr
ikine6s
vellipse

%% JACOBIAN

% JACOB0 Jacobian in world coordinates
% J0 = R.jacob0(Q, OPTIONS) is a 6xN Jacobian matrix for the robot in pose Q.
% The manipulator Jacobian matrix maps joint velocity to end-effector spatial
% velocity V = J0*QD expressed in the world-coordinate frame.

% JACOBN Jacobian in end-effector frame
% JN = R.jacobn(Q, options) is a 6xN Jacobian matrix for the robot in
% pose Q. The manipulator Jacobian matrix maps joint velocity to
% end-effector spatial velocity V = J0*QD in the end-effector frame.

%%TRAJECTORY GENERATION

%JTRAJ Compute a joint space trajectory between two configurations
% [Q,QD,QDD] = JTRAJ(Q0, QF, M) is a joint space trajectory Q (MxN) where the joint
% coordinates vary from Q0 (1xN) to QF (1xN).  A quintic (5th order) polynomial is used
% with default zero boundary conditions for velocity and acceleration.
% Time is assumed to vary from 0 to 1 in M steps.
