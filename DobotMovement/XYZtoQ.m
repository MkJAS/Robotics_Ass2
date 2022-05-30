function [q] = XYZtoQ(point)
    % Using the supplied James Poon method to figure out the Dobot joint angles based off of cartesian XYZ coordinates. Transform required from End Effector to joint 4.
    endEffectorX = point(1);
    endEffectorY = point(2);
    endEffectorZ = point(3);
    theta = atan2(endEffectorX, endEffectorY); %Angle of arm rotation from global jointFourX, assuming robot starts 0 deg facing jointFourX axis

    %*Normalising the angle
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

    %*Joint4 from end effector in end effectors local coords
    jointFourTransformX = 0.05 * sin(theta);
    jointFourTransformY = 0.05 * cos(theta);

    jointFourX = endEffectorX + jointFourTransformX;
    jointFourY = endEffectorY + jointFourTransformY;

    if endEffectorX < 0
        jointFourX = endEffectorX + jointFourTransformX;
    end

    if endEffectorY < 0
        jointFourY = endEffectorY + jointFourTransformY;
    end

    z = (endEffectorZ - (0.138)) + 0.09;

    % z = endEffectorZ + 0.09+0.711547;          %0.09 = end effector length
    % z = z - (0.138+0.711547);           %0.138 = length from base to joint 2

    l = (jointFourX^2 + jointFourY^2)^0.5;
    D = (l^2 + z^2)^0.5;

    joint2LinkA2 = 0.135; %Joint 2 link
    joint3LinkA3 = 0.147; %Joint 3 link

    t1 = rad2deg(atan(z / l));
    t2 = rad2deg(acos((joint2LinkA2^2 + D^2 - joint3LinkA3^2) / (2 * joint2LinkA2 * D)));

    alpha = t1 + t2;

    beta = rad2deg(acos ((joint2LinkA2^2 + joint3LinkA3^2 - D^2) / (2 * joint2LinkA2 * joint3LinkA3)));

    q1 = rad2deg(atan2(jointFourY, jointFourX));
    q2 = (90 - alpha);
    q3 = (180 - beta);
    q4 =- (90 - q2 - q3); %* Dobot Joint 4 also remains perpendicular to the ground
    % q4 = (0 - q3);

    q = deg2rad([q1 q2 q3 q4 0]);
end
