function [collision] = willCollide(robot, q, points)
    % Checks if a point or points will collide with the robot
    % robot = robot
    % q = desired q configuration to be checked
    % points = points that may be in collision
    ellipses = cell(1, 6);
    ellipse_param = zeros(5, 3, 6); %1st row centres, 2nd row radii, 3-5 is rotation matix
    tr = zeros(4, 4, robot.n + 1);
    tr(:, :, 1) = robot.base;
    L = robot.links;

    for i = 1:robot.n
        tr(:, :, i + 1) = tr(:, :, i) * trotz(q(i) + L(i).offset) * transl(0, 0, L(i).d) * transl(L(i).a, 0, 0) * trotx(L(i).alpha);
    end

    for i = 1:5

        if i == 1 || i == 5
            centerPoint = [0, 0, 0];
            z = (robot.links(i).d + robot.links(i).a);
            radii = [0.08, 0.08, z];
            [X, Y, Z] = ellipsoid(centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3));
            a = [X(:), Y(:), Z(:)];
            p1 = tr(1:3, 4, i);
            p2 = tr(1:3, 4, i + 1);
            d = [p2(1) - p1(1), p2(2) - p1(2), p2(3) - p1(3)];
            c = p1 + (d / 2)';
            t = tr(:, :, i);
            xy = ((p2(1) - p1(1))^2 + (p2(2) - p1(2))^2)^0.5;
            t(1:3, 1:3) = t(1:3, 1:3);
            t(1:3, 4) = c;
            b = [t * [a, ones(size(a, 1), 1)]']';
            ellipse_param(1:2, :, i) = [c'; radii];
            ellipse_param(3:5, :, i) = eye(3);
            ellipses{i} = b(:, 1:3);
        else
            centerPoint = [0, 0, 0];
            z = (robot.links(i).d + robot.links(i).a);
            radii = [0.08, 0.08, z];
            [X, Y, Z] = ellipsoid(centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3));
            a = [X(:), Y(:), Z(:)];
            p1 = tr(1:3, 4, i);
            p2 = tr(1:3, 4, i + 1);
            d = [p2(1) - p1(1), p2(2) - p1(2), p2(3) - p1(3)];
            c = p1 + (d / 2)';
            t = tr(:, :, i);
            xy = ((p2(1) - p1(1))^2 + (p2(2) - p1(2))^2)^0.5;
            angle = atan((p2(3) - p2(3) / xy));
            t(1:3, 1:3) = t(1:3, 1:3) * roty(pi / 2) * rotx(-angle);
            t(1:3, 4) = c;
            b = [t * [a, ones(size(a, 1), 1)]']';
            ellipse_param(1:2, :, i) = [c'; radii];
            ellipse_param(3:5, :, i) = t(1:3, 1:3) * roty(pi / 2) * rotx(-angle);
            ellipses{i} = b(:, 1:3);
        end

    end

    for i = 1:6
        %         rot = ellipse_param(3:5,:,i);
        %         trans = ellipse_param(1,:,i);
        t = transl(ellipse_param(1, :, i)');
        t(1:3, 1:3) = ellipse_param(3:5, :, i);
        %                  [inv(tr) * [cubePoints,ones(size(cubePoints,1),1)]']'
        pointsTr = [inv(t) * [points, ones(size(points, 1), 1)]']';
        updatedPoints = pointsTr(:, 1:3);
        centerPoint = [0 0 0];
        radii = [ellipse_param(2, 3, i) ellipse_param(2, 1, i) ellipse_param(2, 2, i)];
        algebraicDist = GetAlgebraicDist(updatedPoints, centerPoint, radii);
        pointsInside = find(algebraicDist < 1);

        if pointsInside >= 1
            collision = true;
            return
        end

    end

end

function algebraicDist = GetAlgebraicDist(points, centerPoint, radii)

    algebraicDist = ((points(:, 1) - centerPoint(1)) / radii(1)).^2 ...
        + ((points(:, 2) - centerPoint(2)) / radii(2)).^2 ...
        + ((points(:, 3) - centerPoint(3)) / radii(3)).^2;
end
