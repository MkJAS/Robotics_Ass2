function GrabAndReturn(robot, object)
    %Uses RMRC to move robot to the object and then return to it's starting position
    %traj = desired trajectory, lims = robot limits
    startPoint = robot.model.fkine(robot.model.getpos());
    startPoint = startPoint(1:3, 4);
    endPoint = object.location;
    endPoint(3) = endPoint(3) + 0.02;

    RMRC(endPoint, 1, robot);
    try delete(object); end
    RMRC(startPoint, 1, robot);
end
