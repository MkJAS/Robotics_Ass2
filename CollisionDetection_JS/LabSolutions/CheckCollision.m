function [bool] = CheckCollision(robot, sphereCenter, radius)
    % function isCollision = CheckCollision(robot, sphereCenter, radius)

    pause(0.1)
    tr = robot.fkine(robot.getpos);
    endEffectorToCenterDist = sqrt(sum((sphereCenter - tr(1:3, 4)').^2));

    if endEffectorToCenterDist <= radius
        disp('Oh no a collision!');
        bool = 1;
    else
        disp(['SAFE: End effector to sphere centre distance (', num2str(endEffectorToCenterDist), 'm) is more than the sphere radius, ' num2str(radius), 'm']);
        bool = 0;
    end

end
