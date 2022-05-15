function [intruder] = lightCurtainCheck(points)
%Checks if an object has passed through light curtains
%   Detailed explanation goes here
intruder = false;

for i=1:size(points,1)
    x = points(i,1);
    y = points(i,2);
    z = points(i,3);

    if x>-0.4 && x<0.4 && y>0.35 && y<0.45 && z<0.45 && z>0
        intruder = true;
        return;
    end 
    if x>-0.4 && x<0.4 && y>-0.45 && y<-0.35 && z<0.45 && z>0
        intruder = true;
        return;
    end 
    if x>-0.45 && x<-0.35 && y>-0.4 && y<0.4 && z<0.45 && z>0
        intruder = true;
        return;
    end 
    if x>0.35 && x<0.45 && y>-0.4 && y<0.4 && z<0.45 && z>0
        intruder = true;
        return;
    end 
end


end