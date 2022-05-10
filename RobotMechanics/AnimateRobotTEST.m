function AnimateRobot(robot, qMatrix)
    %Function that animates one or both of their robots on a desired
    %trajectory. Takes one or two robots, their current joint configurations
    %and their desired joint configurations.

    steps = 100;

    for i = 1:steps
        robot1.model.animate(qMatrix(i, :));
        drawnow();
    end

end
