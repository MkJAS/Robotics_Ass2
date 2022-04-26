function [distance] = Distance(position1,position2)
%Function that finds the distance between two points on the XY plane
    xDiff = position1(1) - position2(1);
    yDiff = position1(2) - position2(2);
    distance = sqrt(xDiff^2 + yDiff^2);
end

