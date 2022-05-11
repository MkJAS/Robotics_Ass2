function PositionObject(robot, placementLocation, objectType)
    %Uses RMRC to move robot to the object and then return to it's starting position
    %traj = desired trajectory, lims = robot limits
    startPoint = robot.model.fkine(robot.model.getpos());
    startPoint = startPoint(1:3, 4);
    endPoint = placementLocation;
    endPoint(3) = endPoint(3) + 0.02;

    RMRC(endPoint, 1, robot);

    switch objectType
        case 'strawberry'
            Strawberry(placementLocation);
        case 'grape'
            Grape(placementLocation);
        case 'pill'
            Pill(placementLocation);
        case 'lego'
            Lego(placementLocation);
    end

    RMRC(startPoint, 1, robot);
end
