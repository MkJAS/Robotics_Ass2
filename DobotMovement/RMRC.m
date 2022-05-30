function [qMatrix] = RMRC(endPoint, trajectoryType, robot)
    %Adapted from lab solutions
    %Uses RMRC to move robot through a specified trajectory
    %trajectoryType = desired trajectory, lims = robot limits
    lims = robot.jointLimits;

    startPoint = robot.model.fkine(robot.model.getpos());
    startPoint = startPoint(1:3, 4);

    totalTime = 5; % (trapTrajScalar)
    deltaT = 0.05; % Control frequency
    steps = totalTime / deltaT; % No. of steps for simulation
    delta = pi / steps; % Small angle change

    epsilonThreshold = 0.01; % Threshold value for manipulability/Damped Least Squares
    weightingMatrix = diag([1 1 1]); % Weighting matrix for the velocity vector

    measureOfManipulability = zeros(steps, 1); % Array for Measure of Manipulability
    qMatrix = zeros(steps, 5); % Array for joint anglesR
    qDot = zeros(steps, 3); % Array for joint velocities
    theta = zeros(3, steps); % Array for roll-pitch-yaw angles
    trajectoryXYZ = zeros(3, steps); % Array for x-y-z trajectory

    trapTrajScalar = lspb(0, 1, steps); % Trapezoidal trajectory scalar.

    switch trajectoryType
        case 1 %straight line

            for i = 1:steps
                trajectoryXYZ(1, i) = (1 - trapTrajScalar(i)) * startPoint(1) + trapTrajScalar(i) * endPoint(1); % Points in x
                trajectoryXYZ(2, i) = (1 - trapTrajScalar(i)) * startPoint(2) + trapTrajScalar(i) * endPoint(2); % Points in y
                trajectoryXYZ(3, i) = (1 - trapTrajScalar(i)) * startPoint(3) + trapTrajScalar(i) * endPoint(3); % Points in z
                theta(1, i) = 0; % Roll angle
                theta(2, i) = 0; % Pitch angle
                theta(3, i) = 0; % Yaw angle
            end

        case 2 %curved line in z plane - vertical
            d2 = pi / steps;

            for i = 1:steps
                trajectoryXYZ(1, i) = (1 - trapTrajScalar(i)) * startPoint(1) + trapTrajScalar(i) * endPoint(1);
                trajectoryXYZ(2, i) = (1 - trapTrajScalar(i)) * startPoint(2) + trapTrajScalar(i) * endPoint(2);
                r = pdist([startPoint'; endPoint]) / 2;
                trajectoryXYZ(3, i) = startPoint(3) + r * sin(i * d2); %0.05 = height
                theta(1, i) = 0; % Roll angle
                theta(2, i) = 0; % Pitch angle
                theta(3, i) = 0; % Yaw angle
            end

        case 3 %curved line in xy plane - horizontal

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

                trajectoryXYZ(1, i) = h + d / 2 * cos(pi / 2 + theta - delta * (i - 1));
                trajectoryXYZ(2, i) = k + d / 2 * sin(pi / 2 + theta - delta * (i - 1));
                trajectoryXYZ(3, i) = (1 - trapTrajScalar(i)) * startPoint(3) + trapTrajScalar(i) * endPoint(3);
                theta(1, i) = 0; % Roll angle
                theta(2, i) = 0; % Pitch angle
                theta(3, i) = 0; % Yaw angle
            end

    end

    qMatrix(1, :) = robot.model.getpos(); % Solve joint angles to achieve first waypoint

    for i = 1:steps - 1
        T = robot.model.fkine(qMatrix(i, :)); % Get forward transformation at current joint state
        deltaX = trajectoryXYZ(:, i + 1) - T(1:3, 4); % Get position error from next waypoint

        %* Don't care about EE orientation
        %         Rnext = rpy2r(theta(1, i + 1), theta(2, i + 1), theta(3, i + 1)); % Get next RPY angles, convert to rotation matrix
        %         Rcurrent = T(1:3, 1:3); % Current end-effector rotation matrix
        %         Rdot = (1 / deltaT) * (Rnext - Rcurrent); % Calculate rotation matrix error
        %         S = Rdot * Rcurrent'; % Skew symmetric!
        %         deltaTheta = tr2rpy(Rnext * Rcurrent'); % Convert rotation matrix to RPY angles

        linear_velocity = (1 / deltaT) * deltaX;
        xdot = weightingMatrix * [linear_velocity]; % Calculate end-effector velocity to reach next waypoint.
        Jacobian = robot.model.jacob0(qMatrix(i, :)); % Get Jacobian at current joint state
        Jacobian = Jacobian(1:3, 1:3); %mask out rpy for Dontbot
        measureOfManipulability(i) = sqrt(det(Jacobian * Jacobian')); %Manipulability - lower = closer to singularity

        if measureOfManipulability(i) < epsilonThreshold % If manipulability is less than given threshold
            lambda = (1 - measureOfManipulability(i) / epsilonThreshold) * 5E-2;

        else
            lambda = 0;
        end

        invertedJacobian = inv(Jacobian' * Jacobian + lambda * eye(3)) * Jacobian'; % Damped Least Squares Inverse
        qDot(i, :) = (invertedJacobian * xdot)'; % Solve the RMRC equation (you may need to transpose the vector)

        qlim = robot.model.qlim;

        for joints = 1:3 % Loop through joints 1 to 6

            if joints == 3
                [~, index] = min(abs(lims(:, 1) - qMatrix(i, 2)));
                [~, index2] = min(abs(lims(:, 3) - qMatrix(i, 2)));
                qlim(3, 1) = lims(index, 2);
                qlim(3, 2) = lims(index2, 4);
            end

            if qMatrix(i, joints) + deltaT * qDot(i, joints) < qlim(joints, 1) % If next joint angle is lower than joint limit...
                qDot(i, joints) = 0; % Stop the motor
            elseif qMatrix(i, joints) + deltaT * qDot(i, joints) > qlim(joints, 2) % If next joint angle is greater than joint limit ...
                qDot(i, joints) = 0; % Stop the motor
            end

        end

        qMatrix(i + 1, 1:3) = qMatrix(i, 1:3) + deltaT * qDot(i, :); % Update next joint state based on joint velocities
        qMatrix(i + 1, 4) =- (pi / 2 - qMatrix(i + 1, 2) - qMatrix(i + 1, 3));

    end

    %! uncomment for plot line on trajectory and to animate within this function
    % plot3(trajectoryXYZ(1, :), trajectoryXYZ(2, :), trajectoryXYZ(3, :), 'k.', 'LineWidth', 1)

    % for i = 1:size(qMatrix, 1)
    %     robot.model.animate(qMatrix(i, :))
    %     pause(0.05);
    % end

end
