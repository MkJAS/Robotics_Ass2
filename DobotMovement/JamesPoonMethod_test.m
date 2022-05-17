close all
clear all
%%
figure (2)
L(1) = Link('d', 0.138, 'a', 0, 'alpha', -pi/2, 'offset', 0,'qlim',[-135*pi/180 135*pi/180]);
L(2) = Link('d', 0, 'a', 0.135, 'alpha',0,'offset', -pi/2,'qlim',[5*pi/180 80*pi/180]);
L(3) = Link('d', 0, 'a', 0.147, 'alpha', pi, 'offset',0,'qlim',[5*pi/180 85*pi/180]);
L(4) = Link('d', 0, 'a', 0.05, 'alpha', pi/2, 'offset', 0,'qlim',[-pi/2 pi/2]);
L(5) = Link('d', 0.09, 'a', 0, 'alpha',0, 'offset', 0,'qlim',[-85*pi/180 85*pi/180]);

robot = SerialLink(L);
% robot = Dobot;


q = zeros(1,5);
robot.plot(deg2rad([0 40 60 10 0]));
hold on
% robot.teach;

%%

% steps = deg2rad(5);        %revolute joint increments
% 
% qlim = robot.qlim;
% 
% qlim(2,:) = [5*pi/180 80*pi/180];
% qlim(3,:) = [5*pi/180 85*pi/180];
% 
% %Get the total size of the pointClouds
% pointCloud1size = prod(floor((qlim(1:5,2)-qlim(1:5,1)/steps + 1)));
% pointCloud1 = zeros(pointCloud1size,3);
% counter = 1;
% 
% lowerLimit = [];
% upperLimit = [];
%  for q2 = qlim(2,1):0.01:qlim(2,2)
%      for theta3 = qlim(3,1):0.01:qlim(3,2)+0.01
%          q3 = pi/2 - q2 + theta3;
%          if theta3 <= qlim(3,1)
%             lowerLimit = [lowerLimit;q2,q3]; %#ok<AGROW>
%          elseif qlim(3,2) <= theta3
%             upperLimit = [upperLimit;q2,q3]; %#ok<AGROW>
%          end
%      end
%  end
% % 
% % figure (2)
% % 
% % plot(rad2deg(lowerLimit(:,1)),rad2deg(lowerLimit(:,2)),'b');
% % hold on;
% % plot(rad2deg(upperLimit(:,1)),rad2deg(upperLimit(:,2)),'r');
% 
%        
% qlim = robot.qlim;
% 
% 
%     for q1 = qlim(1,1):steps:qlim(1,2)
%         for q2 = qlim(2,1):steps:qlim(2,2)
%             [~, index] = min(abs(lowerLimit(:,1)-q2));
%             [~, index2] = min(abs(upperLimit(:,1)-q2));
%             LL = round(rad2deg(lowerLimit(index,2))/5)*5;
%             UL = round(rad2deg(upperLimit(index2,2))/5)*5;
%             LL = deg2rad(LL);
%             UL = deg2rad(UL);
%             for q3 = LL:steps:UL
%                 q4 = -(pi/2 - q2 - q3);
%                 q5 = 0;
%                 q = [q1,q2,q3,q4,q5];
%                 tr = robot.fkine(q);                        
%                 pointCloud1(counter,:) = tr(1:3,4)';
%                 counter = counter + 1;
%             end 
%         end
%     end
% 
% 
% r1cloud = plot3(pointCloud1(:,1),pointCloud1(:,2),pointCloud1(:,3),'r.');
% pause(0.5);
% try delete(r1cloud); end;
% 



%%
robot.plot(deg2rad([0 40 60 10 0]));

q = robot.getpos();
startPoint = robot.fkine(q);
startPoint = startPoint(1:3,4);
endPoint = startPoint;
endPoint(1) = endPoint(1) + 0.01;

%End effector pos
Ex = endPoint(1);
Ey = endPoint(2);
Ez = endPoint(3);
qt = XYZtoQ(endPoint);
theta = atan(Ey/Ex);         %Angle of arm rotation from global x, assuming robot starts 0 deg facing x axis

Tx = 0.05*cos(theta);        %Joint4 from end effector in end effectors local coords
Ty = 0.05*sin(theta);

%Joint4 pos
x = Ex - Tx;
y = Ey - Ty;


if Ex < 0
    x = Ex + Tx;
end
if Ey < 0
    y = Ey + Ty;
end 

z = Ez + 0.09;          %0.09 = end effector length
z = z - 0.138;           %0.138 = length from base to joint 2


l = (x^2+y^2)^0.5;
D = (l^2+z^2)^0.5;

a2 = 0.135;             %Joint 2 link
a3 = 0.147;             %Joint 3 link


t1 = rad2deg(atan(z/l));

t2 = rad2deg(acos((a2^2 + D^2 - a3^2)/(2*a2*D)));

alpha = t1 + t2;

beta = rad2deg( acos ( ( a2^2 + a3^2 - D^2) / (2*a2*a3) ) );

q1 = rad2deg(atan2(y,x));

q2 =  (90 - alpha);

q3r = (180 - beta);

% q4 = (0 - q3r);

q4 = -(90 - q2 - q3r);
q = deg2rad([q1 q2 q3r q4 0])

robot.plot(deg2rad([q1 q2 q3r q4 0]))
% t = robot.fkine(robot.getpos());

% t(1:3,4)



% tr = zeros(4,4,robot.n+1);
% tr(:,:,1) = robot.base;
% L = robot.links;
% for i = 1 : robot.n
%     tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
% end


%%
% figure (1)
% 
% t = 2.5;             % Total time (s)
% deltaT = 0.05;      % Control frequency
% steps = t/deltaT;   % No. of steps for simulation
% delta = pi/steps; % Small angle change
% d2 = 2*pi/steps;
% epsilon = 0.01;      % Threshold value for manipulability/Damped Least Squares
% W = diag([1 1 1]);    % Weighting matrix for the velocity vector
% 
% % 1.2) Allocate array data
% m = zeros(steps,1);             % Array for Measure of Manipulability
% qMatrix = zeros(steps,5);       % Array for joint anglesR
% qdot = zeros(steps,3);          % Array for joint velocities
% theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
% x = zeros(3,steps);             % Array for x-y-z trajectory
% positionError = zeros(3,steps); % For plotting trajectory error
% angleError = zeros(3,steps);    % For plotting trajectory error
% 
% % 1.3) Set up trajectory, initial pose
% s = lspb(0,1,steps);                % Trapezoidal trajectory scalar
% for i=1:steps
%     
%     x(1,i) = (1-s(i))*0.2 + s(i)*0.2;
%     x(2,i) = (1-s(i))*-0.1 + s(i)*0.1;                                       % Points in x
%     x(3,i) = (1-s(i))*0.094 + s(i)*0.15;                                          % Points in y
% %     x(3,i) = 0.1 + 0.05*sin(i*d2);                                       % Points in z
%     theta(1,i) = 0;                 % Roll angle 
%     theta(2,i) = 0;                 % Pitch angle
%     theta(3,i) = 0;                 % Yaw angle
% end
%  
% T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];          % Create transformation of first point and angle
% q0 = zeros(1,5);                                                            % Initial guess for joint angles
% qMatrix(1,:) = q;                                            % Solve joint angles to achieve first waypoint
% 
% % 1.4) Track the trajectory with RMRC
% for i = 1:steps-1
%     T = robot.fkine(qMatrix(i,:));                                           % Get forward transformation at current joint state
%     deltaX = x(:,i+1) - T(1:3,4);                                       	% Get position error from next waypoint
%     Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
%     Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
%     Rdot = (1/deltaT)*(Rd - Ra);                                                % Calculate rotation matrix error
%     S = Rdot*Ra';                                                           % Skew symmetric!
%     linear_velocity = (1/deltaT)*deltaX;
%     angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
%     deltaTheta = tr2rpy(Rd*Ra');                                            % Convert rotation matrix to RPY angles
%     xdot = W*[linear_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
%     J = robot.jacob0(qMatrix(i,:));                                          % Get Jacobian at current joint state
%     J = J(1:3,1:3);
%     m(i) = sqrt(det(J*J'));
%     if m(i) < epsilon                                      % If manipulability is less than given threshold
%         lambda = (1 - m(i)/epsilon)*5E-2;
%         
%     else
%         lambda = 0;
%     end
%     invJ = inv(J'*J + lambda *eye(3))*J';                                   % DLS Inverse
%     qdot(i,:) = (invJ*xdot)';                                                % Solve the RMRC equation (you may need to transpose the         vector)
%     
%     qlim = robot.qlim;
%     
%     
%     for j = 1:3                                                             % Loop through joints 1 to 6
%         if j == 3
%             [~, index] = min(abs(lowerLimit(:,1)-qMatrix(i,2)));
%             [~, index2] = min(abs(upperLimit(:,1)-qMatrix(i,2)));
%             qlim(3,1) = lowerLimit(index,2);
%             qlim(3,2) = upperLimit(index2,2);
%         end 
%         if qMatrix(i,j) + deltaT*qdot(i,j) < qlim(j,1)                % If next joint angle is lower than joint limit...
%             qdot(i,j) = 0; % Stop the motor
%         elseif qMatrix(i,j) + deltaT*qdot(i,j) > qlim(j,2)            % If next joint angle is greater than joint limit ...
%             qdot(i,j) = 0; % Stop the motor
%         end
%     end
%     qMatrix(i+1,1:3) = qMatrix(i,1:3) + deltaT*qdot(i,:);                       % Update next joint state based on joint velocities
%     qMatrix(i+1,4) = -(pi/2 - qMatrix(i+1,2) - qMatrix(i+1,3));
%     
%     positionError(:,i) = x(:,i+1) - T(1:3,4);                               % For plotting
%     angleError(:,i) = deltaTheta;                                           % For plotting
% end
% 
% 
% %%
% plot3(x(1,:),x(2,:),x(3,:),'k.','LineWidth',1)
% for i=1:size(qMatrix,1)
%     robot.plot(qMatrix(i,:))
%     pause(0.05);
% 
% end
% 
% 
% 
% %%
% for i = 1:3
%     figure(2)
%     subplot(3,2,i)
%     plot(qMatrix(:,i),'k','LineWidth',1)
%     title(['Joint ', num2str(i)])
%     ylabel('Angle (rad)')
%       if i==3
%           hold on
%           plot([1:131],lowerLimit(:,2))
%           plot([1:131],upperLimit(:,2))
%           hold off
%       else
%     refline(0,robot.qlim(i,1));
%     refline(0,robot.qlim(i,2));
%       end
%     
%     
%     figure(3)
%     subplot(3,2,i)
%     plot(qdot(:,i),'k','LineWidth',1)
%     title(['Joint ',num2str(i)]);
%     ylabel('Velocity (rad/s)')
%     refline(0,0)
% end
% 
% figure(4)
% subplot(2,1,1)
% plot(positionError'*1000,'LineWidth',1)
% refline(0,0)
% xlabel('Step')
% ylabel('Position Error (mm)')
% legend('X-Axis','Y-Axis','Z-Axis')
% 
% subplot(2,1,2)
% plot(angleError','LineWidth',1)
% refline(0,0)
% xlabel('Step')
% ylabel('Angle Error (rad)')
% legend('Roll','Pitch','Yaw')
% 
% figure(5)
% plot(m,'k','LineWidth',1)
% refline(0,epsilon)
% title('Manipulability')
% 
% robot.teach();
% 
% %%
% % 
% 
% ellipses = cell(1,6);
%     ellipse_param = zeros(5,3,6);   %1st row centres, 2nd row radii, 3-5 is rotation matix
%     
%     tr = zeros(4,4,robot.n+1);
%     tr(:,:,1) = robot.base;
%     L = robot.links;
%     for i = 1 : robot.n
%         tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
%     end
%     for i = 1:5
%         if i == 1 || i == 5
%             centerPoint = [0,0,0];
%             z = (robot.links(i).d+robot.links(i).a);
%             radii = [0.08,0.08,z];
%             [X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );
%             a = [X(:),Y(:),Z(:)];
%             p1 = tr(1:3,4,i);
%             p2 = tr(1:3,4,i+1);
%             d = [p2(1) - p1(1), p2(2) - p1(2), p2(3) - p1(3)];
%             c = p1 + (d/2)';
%             t = tr(:,:,i);
%             xy = ((p2(1) - p1(1))^2 + (p2(2) - p1(2))^2)^0.5;
%             t(1:3,1:3) = t(1:3,1:3);
%             t(1:3,4) = c;
%             b = [t*[a,ones(size(a,1),1)]']';
%             ellipse_param(1:2,:,i) = [c';radii];
%             ellipse_param(3:5,:,i) = eye(3);
%             ellipses{i} = b(:,1:3);
%         else
%             centerPoint = [0,0,0];
%             z = (robot.links(i).d+robot.links(i).a);
%             radii = [0.08,0.08,z];
%             [X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );
%             a = [X(:),Y(:),Z(:)];
%             p1 = tr(1:3,4,i);
%             p2 = tr(1:3,4,i+1);
%             d = [p2(1) - p1(1), p2(2) - p1(2), p2(3) - p1(3)];
%             c = p1 + (d/2)';
%             t = tr(:,:,i);
%             xy = ((p2(1) - p1(1))^2 + (p2(2) - p1(2))^2)^0.5;
%             angle = atan((p2(3)-p2(3)/xy));
%             t(1:3,1:3) = t(1:3,1:3)* roty(pi/2)*rotx(-angle);
%             t(1:3,4) = c;
%             b = [t*[a,ones(size(a,1),1)]']';
%             ellipse_param(1:2,:,i) = [c';radii];
%             ellipse_param(3:5,:,i) = t(1:3,1:3)*roty(pi/2)*rotx(-angle);
%             ellipses{i} = b(:,1:3);
%         end
%     end   
% % robot.plot3d(q);
% 
% % 
% % t = tr(:,:,2);
% % a = tr(:,:,2);
% % figure (3)
% % hold on
% % a * transl[0,0,0];
% % a * transl(0,0,0);
% % trplot(a);
% % t(1:3,1:3) = t(1:3,1:3)* roty(pi/2)*rotx(pi/4);
% % tr(1:3,4) = [0;0;0];
% % figure (2)
% % hold on
% % b = [t*[a,ones(size(a,1),1)]']';
% % b = b(:,1:3);
% % scatter3(b(:,1),b(:,2),b(:,3))
% % t
% % a = [robot.points{2}(:,1) robot.points{2}(:,2) robot.points{2}(:,3)];
% % b = [t*[a,ones(size(a,1),1)]']';
% % b = b(:,1:3);
% % scatter3(b(:,1),b(:,2),b(:,3))



