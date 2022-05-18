function PickupObject(robot, object, points)
    %Uses RMRC to move robot to the object and then return to it's starting position
    %traj = desired trajectory, lims = robot limits

    midPoint = object.location;
    midPoint(3) = midPoint(3) + 0.02;

    endPoint = midPoint;
    endPoint(3) = endPoint(3) + 0.1;

    qMatrix = RMRC(midPoint, 1, robot);

    for i = 1:size(qMatrix, 1)
        collision = willCollide(robot, qMatrix(i, :), points);
        intruder = lightCurtainCheck(handles.pcPoints);

        if collision == true
            cprintf('red', 'Possible collision detected! Aborting move!\n');
            beep
            cprintf('red', 'Waiting for obstacle to be removed\n'); s
        end

        robot.model.animate(qMatrix(i, :));
        vertices = object.vertices;
        tr = robot.model.fkine(qMatrix(i, :));
        transformedVertices = [vertices, ones(size(vertices, 1), 1)] * tr';
        set(mesh_h, 'Vertices', transformedVertices(:, 1:3));
        drawnow();
    end

    try delete(object); end
    qMatrix = RMRC(endPoint, 1, robot);
end
