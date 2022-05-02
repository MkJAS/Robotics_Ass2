function [] = Lab5Starter()
    cla
    set(0, 'DefaultFigureWindowStyle', 'docked')
    clc
    close all

    %% Create 1-link robot
    L1 = Link('d', 0, 'a', 1, 'alpha', 0, 'qlim', [-pi pi]);
    robot = SerialLink(L1, 'name', 'myRobot');

    q = 0;
    scale = 0.5;
    workspace = [-0.5 1.5 -0.5 1.5 -1 1];
    robot.plot(q, 'workspace', workspace, 'scale', scale);
    hold on;

    %% Create sphere
    sphereCenter = [1, 1, 0];
    radius = 0.5;
    [X, Y, Z] = sphere(20);
    X = X * radius + sphereCenter(1);
    Y = Y * radius + sphereCenter(2);
    Z = Z * radius + sphereCenter(3);

    %% Plot it
    % Plot point cloud
    points = [X(:), Y(:), Z(:)];
    spherePc_h = plot3(points(:, 1), points(:, 2), points(:, 3), 'r.'); pause
    delete (spherePc_h)

    % % Or a triangle mesh
    tri = delaunay(X, Y, Z);
    sphereTri_h = trimesh(tri, X, Y, Z);
    drawnow();
    view(3)
    axis equal

    %% Move Robot
    for q = 0:pi / 180:pi / 2
        robot.plot(q);
        drawnow();
        CheckCollision(robot, sphereCenter, radius);

        if CheckCollision(robot, sphereCenter, radius) == 1
            disp('UNSAFE: Robot stopped')
            break
        end

    end
