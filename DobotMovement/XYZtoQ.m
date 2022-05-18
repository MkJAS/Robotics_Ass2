function [q] = XYZtoQ(point)
    % Using the supplied James Poon method to figure out the Dobot joint angles based off of cartesian XYZ coordinates. Transform required from End Effector to joint 4.
    Ex = point(1);
    Ey = point(2);
    Ez = point(3);
    theta = atan2(Ex, Ey); %Angle of arm rotation from global x, assuming robot starts 0 deg facing x axis

    if theta > pi / 2
        theta = pi - theta;
        theta = 2 * pi - theta;
    end

    if theta > 0 && theta < pi / 2
        theta = pi + theta;
    end

    if theta <- pi / 2
        theta = pi + theta;
    end

    if theta < 0 && theta >- pi / 2
        theta = pi + theta;
    end

    Ty = 0.05 * cos(theta); %Joint4 from end effector in end effectors local coords
    Tx = 0.05 * sin(theta);

    %Joint4 pos
    x = Ex + Tx;
    y = Ey + Ty;

    if Ex < 0
        x = Ex + Tx;
    end

    if Ey < 0
        y = Ey + Ty;
    end

    z = (Ez - (0.138)) + 0.09;

    % z = Ez + 0.09+0.711547;          %0.09 = end effector length
    % z = z - (0.138+0.711547);           %0.138 = length from base to joint 2

    l = (x^2 + y^2)^0.5;
    D = (l^2 + z^2)^0.5;

    a2 = 0.135; %Joint 2 link
    a3 = 0.147; %Joint 3 link

    t1 = rad2deg(atan(z / l));

    t2 = rad2deg(acos((a2^2 + D^2 - a3^2) / (2 * a2 * D)));

    alpha = t1 + t2;

    beta = rad2deg(acos ((a2^2 + a3^2 - D^2) / (2 * a2 * a3)));

    q1 = rad2deg(atan2(y, x));

    q2 = (90 - alpha);

    q3r = (180 - beta);

    % q4 = (0 - q3r);

    q4 =- (90 - q2 - q3r);

    q = deg2rad([q1 q2 q3r q4 0]);
end
