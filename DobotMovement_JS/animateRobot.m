function animateRobot(robot,q,object)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    if nargin < 3
        robot.model.animate(q);
        pause(0.01);
    else   
        robot.model.animate(q);
        vertices = object.vertices;
        tr = robot.model.fkine(q);
        tr = transl(tr(1:3,4));
        tr(1,4) = tr(1,4) - 0.2;
        tr(2,4) = tr(2,4) + 0.2;
        transformedVertices = [vertices,ones(size(vertices,1),1)] * tr';
        set(object.object,'Vertices',transformedVertices(:,1:3));
        pause(0.01);
    end 
end