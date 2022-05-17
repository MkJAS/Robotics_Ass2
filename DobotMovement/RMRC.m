function [qMatrix] = RMRC(endPoint, traj, robot)
    %Uses RMRC to move robot through a specified trajectory
    %traj = desired trajectory, lims = robot limits
    lims = robot.jointLimits;

    startPoint = robot.model.fkine(robot.model.getpos());
    startPoint = startPoint(1:3, 4);




    %%
    % figure (1)

    t = 5; % Total time (s)
    deltaT = 0.05; % Control frequency
    steps = 70; % steps = t / deltaT; % No. of steps for simulation
    delta = pi / steps; % Small angle change

    epsilon = 0.01; % Threshold value for manipulability/Damped Least Squares
    W = diag([1 1 1]); % Weighting matrix for the velocity vector

    % 1.2) Allocate array data
    m = zeros(steps, 1); % Array for Measure of Manipulability
    qMatrix = zeros(steps, 5); % Array for joint anglesR
    qdot = zeros(steps, 3); % Array for joint velocities
    theta = zeros(3, steps); % Array for roll-pitch-yaw angles
    xyz = zeros(3, steps); % Array for x-y-z trajectory
%     positionError = zeros(3, steps); % For plotting trajectory error
%     angleError = zeros(3, steps); % For plotting trajectory error

    % 1.3) Set up trajectory, initial pose
    s = lspb(0, 1, steps); % Trapezoidal trajectory scalar.

    switch traj
        case 1              %trajectory 1 = straight line
            for i = 1:steps
                xyz(1, i) = (1 - s(i)) * startPoint(1) + s(i) * endPoint(1); % Points in x
                xyz(2, i) = (1 - s(i)) * startPoint(2) + s(i) * endPoint(2); % Points in y
                xyz(3, i) = (1 - s(i)) * startPoint(3) + s(i) * endPoint(3); % Points in z
                theta(1, i) = 0; % Roll angle
                theta(2, i) = 0; % Pitch angle
                theta(3, i) = 0; % Yaw angle
            end

        case 2                  %trajectory 2 = curved line in z plane
            d2 = pi / steps;
            for i = 1:steps     
                xyz(1, i) = (1 - s(i)) * startPoint(1) + s(i) * endPoint(1);
                xyz(2, i) = (1 - s(i)) * startPoint(2) + s(i) * endPoint(2);
                r = pdist([startPoint';endPoint])/2;
                xyz(3, i) = startPoint(3) + r * sin(i * d2); %0.05 = height
                theta(1, i) = 0; % Roll angle
                theta(2, i) = 0; % Pitch angle
                theta(3, i) = 0; % Yaw angle
            end

        case 3          %trajectory 3 = curved line in xy plane
            for i = 1:steps
                d2 = pi / steps;
                dx = endPoint(1) - startPoint(1);
                dy = endPoint(2) - startPoint(2);
                d = (dx^2 + dy^2)^0.5;
                h = dx / 2 + startPoint(1);
                k = dy / 2 + startPoint(2);
                theta = atan2(k, h);
                if theta < 0
                    theta = theta + 2 * pi;
                end
                xyz(1, i) = h + d / 2 * cos(pi / 2 + theta - delta * (i - 1));
                xyz(2, i) = k + d / 2 * sin(pi / 2 + theta - delta * (i - 1));
                xyz(3, i) = (1 - s(i)) * startPoint(3) + s(i) * endPoint(3);
                theta(1, i) = 0; % Roll angle
                theta(2, i) = 0; % Pitch angle
                theta(3, i) = 0; % Yaw angle
            end

    end

    % T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) xyz(:,1);zeros(1,3) 1];          % Create transformation of first point and angle
    % q0 = zeros(1,5);                                                            % Initial guess for joint angles
    qMatrix(1, :) = robot.model.getpos(); % Solve joint angles to achieve first waypoint

    % 1.4) Track the trajectory with RMRC
    for i = 1:steps - 1
        T = robot.model.fkine(qMatrix(i, :)); % Get forward transformation at current joint state
        deltaX = xyz(:, i + 1) - T(1:3, 4); % Get position error from next waypoint
        Rd = rpy2r(theta(1, i + 1), theta(2, i + 1), theta(3, i + 1)); % Get next RPY angles, convert to rotation matrix
        Ra = T(1:3, 1:3); % Current end-effector rotation matrix
        Rdot = (1 / deltaT) * (Rd - Ra); % Calculate rotation matrix error
        S = Rdot * Ra'; % Skew symmetric!
        linear_velocity = (1 / deltaT) * deltaX;
        %     angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
        deltaTheta = tr2rpy(Rd * Ra'); % Convert rotation matrix to RPY angles
        xdot = W * [linear_velocity]; % Calculate end-effector velocity to reach next waypoint.
        J = robot.model.jacob0(qMatrix(i, :)); % Get Jacobian at current joint state
        J = J(1:3, 1:3);
        m(i) = sqrt(det(J * J'));

        if m(i) < epsilon % If manipulability is less than given threshold
            lambda = (1 - m(i) / epsilon) * 5E-2;

        else
            lambda = 0;
        end

        invJ = inv(J' * J + lambda * eye(3)) * J'; % DLS Inverse
        qdot(i, :) = (invJ * xdot)'; % Solve the RMRC equation (you may need to transpose the         vector)

        qlim = robot.model.qlim;

        for j = 1:3 % Loop through joints 1 to 6

            if j == 3
                [~, index] = min(abs(lims(:, 1) - qMatrix(i, 2)));
                [~, index2] = min(abs(lims(:, 3) - qMatrix(i, 2)));
                qlim(3, 1) = lims(index, 2);
                qlim(3, 2) = lims(index2, 4);
            end

            if qMatrix(i, j) + deltaT * qdot(i, j) < qlim(j, 1) % If next joint angle is lower than joint limit...
                qdot(i, j) = 0; % Stop the motor
            elseif qMatrix(i, j) + deltaT * qdot(i, j) > qlim(j, 2) % If next joint angle is greater than joint limit ...
                qdot(i, j) = 0; % Stop the motor
            end

        end

        qMatrix(i + 1, 1:3) = qMatrix(i, 1:3) + deltaT * qdot(i, :); % Update next joint state based on joint velocities
        qMatrix(i + 1, 4) =- (pi / 2 - qMatrix(i + 1, 2) - qMatrix(i + 1, 3));

        %     positionError(:,i) = xyz(:,i+1) - T(1:3,4);                               % For plotting
        %     angleError(:,i) = deltaTheta;                                           % For plotting
    end

    %! uncomment for plot line on trajectory
    % plot3(xyz(1, :), xyz(2, :), xyz(3, :), 'k.', 'LineWidth', 1)
end
