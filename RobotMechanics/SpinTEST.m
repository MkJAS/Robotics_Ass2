function Spin(robot)
    %Function that animates a robot rotating to a desired joint state
    %Takes a robot and the desired joint configuration.
    % angle needs to be -135 -> 135 degrees
    while 1
        AnimateRobots(robot, robot.qIntermediary);

        steps = 50;

        qCurrent = robot.model.getpos();
        qTarget = qCurrent;
        qTarget(1) = deg2rad(-135);

        %Find q trajectory
        qTrajectory = jtraj(qCurrent, qTarget, steps); %(current q, target q, steps)

        for i = 1:steps
            robot.model.animate(qTrajectory(i, :));
            drawnow();
        end

        qCurrent = robot.model.getpos();
        qTarget = qCurrent;
        qTarget(1) = deg2rad(135);

        %Find q trajectory
        qTrajectory = jtraj(qCurrent, qTarget, steps); %(current q, target q, steps)

        for i = 1:steps
            robot.model.animate(qTrajectory(i, :));
            drawnow();
        end

    end

end
