function AnimateRobots(logFile, robot1, qCurrent1, qTarget1, robot2, qCurrent2, qTarget2)
    %Function that animates one or both of their robots on a desired
    %trajectory. Takes one or two robots, their current joint configurations
    %and their desired joint configurations.

    steps = 50;

    switch (nargin)
        case 4
            %Log Transforms
            EasyLogger(logFile, robot1, qCurrent1, qTarget1);

            %Find q trajectory %* Implement
            qTrajectory1 = jtraj(qCurrent1, qTarget1, steps); %(current q, target q, steps)

            for i = 1:steps
                robot1.model.animate(qTrajectory1(i, :));
                drawnow();
            end

        case 7
            %Log Transforms
            EasyLogger(logFile, robot1, qCurrent1, qTarget1);
            EasyLogger(logFile, robot2, qCurrent2, qTarget2);

            %Find q trajectory
            qTrajectory1 = jtraj(qCurrent1, qTarget1, steps); %(current q, target q, steps)
            qTrajectory2 = jtraj(qCurrent2, qTarget2, steps); %(current q, target q, steps)

            for i = 1:steps
                robot1.model.animate(qTrajectory1(i, :));
                robot2.model.animate(qTrajectory2(i, :));
                drawnow();
            end

        otherwise
            error('This number of arguments is not supported')
    end

end
