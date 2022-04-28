close all
clear all


L(1) = Link('d', 0.138, 'a', 0, 'alpha', -pi/2, 'offset', 0,'qlim',[-135*pi/180 135*pi/180]);
            L(2) = Link('d', 0, 'a', 0.135, 'alpha',0,'offset', -pi/2,'qlim',[-5*pi/180 80*pi/180]);
            L(3) = Link('d', 0, 'a', 0.147, 'alpha', pi, 'offset',0,'qlim',[-5*pi/180 85*pi/180]);
            L(4) = Link('d', 0, 'a', 0.05, 'alpha', pi/2, 'offset', 0,'qlim',[-pi/2 pi/2]);
            L(5) = Link('d', 0.09, 'a', 0, 'alpha',0, 'offset', 0,'qlim',[-85*pi/180 85*pi/180]);
% 
% L5 = Link('d',0,'a',0,'alpha',-pi/2,'offset',0);
% 
% L6 = Link('d',0,'a',0,'alpha',0,'offset',0);

robot = SerialLink(L);
% robot = Dobot;


q = zeros(1,5);
robot.plot(q);
robot.teach;


%%
clc 

%End effector pos
Ex = 0.22
Ey = 0
Ez = 0.1

theta = atan(Ey/Ex)         %Angle of arm rotation from global x, assuming robot starts 0 deg facing x axis

Tx = 0.05*cos(theta)        %Joint4 from end effector in end effectors local coords
Ty = 0.05*sin(theta)

%Joint4 pos
x = Ex - Tx
y = Ey - Ty


if Ex < 0
    x = Ex + Tx
end
if Ey < 0
    y = Ey + Ty
end 

z = Ez + 0.09;          %0.09 = end effector length
z = z - 0.138           %0.138 = length from base to joint 2


l = (x^2+y^2)^0.5
D = (l^2+z^2)^0.5

a2 = 0.135;             %Joint 2 link
a3 = 0.147;             %Joint 3 link


t1 = rad2deg(atan(z/l))

t2 = rad2deg(acos((a2^2 + D^2 - a3^2)/(2*a2*D)))

alpha = t1 + t2

beta = rad2deg( acos ( ( a2^2 + a3^2 - D^2) / (2*a2*a3) ) )

q1 = rad2deg(atan(y/x))

q2 =  (90 - alpha)

q3r = (180 - beta)

% q4 = (0 - q3r)

q4 = -(90 - q2 - q3r)

robot.plot(deg2rad([q1 q2 q3r q4 0]));

t = robot.fkine(robot.getpos());

t(1:3,4)

q = deg2rad([q1 q2 q3r q4 0]);

tr = zeros(4,4,robot.n+1);
tr(:,:,1) = robot.base;
L = robot.links;
for i = 1 : robot.n
    tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
end
tr

%%
figure (2)

d = Dobot;

d.model.animate(deg2rad([q1 q2 q3r q4 0]))






