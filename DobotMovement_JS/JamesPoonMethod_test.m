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


x = 0.25;
y = 0;
z = 0;


l = (x^2+y^2)^0.5
D = (l^2+z^2)^0.5

a2 = 0.135;
a3 = 0.147;


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

%%
figure (2)

d = Dobot;

d.model.animate(deg2rad([q1 q2 q3r q4 0]))






