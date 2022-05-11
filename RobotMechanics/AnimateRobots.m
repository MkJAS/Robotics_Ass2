function AnimateRobots(robot, qTarget)
    %Function that animates one or both of their robots on a desired
    %trajectory. Takes one or two robots, their current joint configurations
    %and their desired joint configurations.

    steps = 100;
    qCurrent = robot.model.getpos();

    %Find q trajectory
    qTrajectory = jtraj(qCurrent, qTarget, steps); %(current q, target q, steps)

    for i = 1:steps
        robot.model.animate(qTrajectory(i, :));
        drawnow();
    end

end

% transformFruit = robot.model.fkine(qTrajectory(i, :));
% verticesTransformed = [fruit.vertices, ones(size(fruit.vertices, 1), 1)] * transformFruit';
% set(fruit.self, 'Vertices', verticesTransformed(:, 1:3));
