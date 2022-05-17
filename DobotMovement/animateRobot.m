function animateRobot(robot,q,object)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    if nargin < 3
        robot.model.animate(q);
        pause(0.01);
    else   
    switch object.type
        case 'strawberry'
        robot.model.animate(q);
        vertices = object.vertices;
        tr = robot.model.fkine(q);
        tr = transl(tr(1:3,4));
        tr(1,4) = tr(1,4) - 0.2;
        tr(2,4) = tr(2,4) + 0.2;
        transformedVertices = [vertices,ones(size(vertices,1),1)] * tr';
        set(object.object,'Vertices',transformedVertices(:,1:3));
        pause(0.01);
        case 'grape'
        robot.model.animate(q);
        vertices = object.vertices;
        tr = robot.model.fkine(q);
        tr = transl(tr(1:3,4));
        tr(1,4) = tr(1,4) - 0.05;
        tr(2,4) = tr(2,4) + 0.175;
        tr(3,4) = tr(3,4) - 0.015;
        transformedVertices = [vertices,ones(size(vertices,1),1)] * tr';
        set(object.object,'Vertices',transformedVertices(:,1:3));
        pause(0.01);
        case 'lego'
        robot.model.animate(q);
        vertices = object.vertices;
        tr = robot.model.fkine(q);
        tr = transl(tr(1:3,4));
        tr(1,4) = tr(1,4) - 0.1;
        tr(2,4) = tr(2,4) + 0.25;
        tr(3,4) = tr(3,4) - 0.015;
        transformedVertices = [vertices,ones(size(vertices,1),1)] * tr';
        set(object.object,'Vertices',transformedVertices(:,1:3));
        pause(0.01);
        case 'orangelego'
        robot.model.animate(q);
        vertices = object.vertices;
        tr = robot.model.fkine(q);
        tr = transl(tr(1:3,4));
        tr(1,4) = tr(1,4) - 0.21;
        tr(2,4) = tr(2,4) + 0.06;
        tr(3,4) = tr(3,4) - 0.015;
        transformedVertices = [vertices,ones(size(vertices,1),1)] * tr';
        set(object.object,'Vertices',transformedVertices(:,1:3));
        pause(0.01);
        case 'yellowlego'
        robot.model.animate(q);
        vertices = object.vertices;
        tr = robot.model.fkine(q);
        tr = transl(tr(1:3,4));
        tr(1,4) = tr(1,4) - 0.17;
        tr(2,4) = tr(2,4) + 0.15;
        tr(3,4) = tr(3,4) - 0.015;
        transformedVertices = [vertices,ones(size(vertices,1),1)] * tr';
        set(object.object,'Vertices',transformedVertices(:,1:3));
        pause(0.01);
    end 
end