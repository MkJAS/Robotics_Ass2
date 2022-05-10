function RotateRobot(logFile, robot, angle)
    %Function that animates a robot rotating to a desired joint state
    %Takes a robot and the desired joint configuration.

    steps = 100;

    qCurrent = robot.model.getpos();
    qTarget = qCurrent;
    qTarget(1) = deg2rad(angle);

    %Log Transforms
    EasyLogger(logFile, robot, qCurrent, qTarget);

    %Find q trajectory
    qTrajectory = jtraj(qCurrent, qTarget, steps); %(current q, target q, steps)

    for i = 1:steps
        robot.model.animate(qTrajectory(i, :));
        drawnow();
    end

end
