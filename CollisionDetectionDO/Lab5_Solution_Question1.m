%% Robotics
% Lab 5 - Question 1 - UFO Blasting
function [  ] = Lab5Solution_Question1(  )
clf

% Add UFO Fleet
ufoFleet = UFOFleet(10);
% Add blasting robot
blasterRobot = SchunkUTSv2_0();
plot3d(blasterRobot,zeros(1,6));

% Make a blasting cone out of the end effector        
endEffectorTr = blasterRobot.fkine(zeros(1,6));
blasterRobot.delay = 0;
[X,Y,Z] = cylinder([0,0.1],6);
Z = Z * 10;
updatedConePoints = [endEffectorTr * [X(:),Y(:),Z(:),ones(numel(X),1)]']';
conePointsSize = size(X);
cone_h = surf(reshape(updatedConePoints(:,1),conePointsSize) ...
             ,reshape(updatedConePoints(:,2),conePointsSize) ...
             ,reshape(updatedConePoints(:,3),conePointsSize));
view(3);

% Plot the point score board
currentScore = 0;
scoreZ = ufoFleet.workspaceDimensions(end)*1.2;
text_h = text(0, 0, scoreZ,sprintf('Score: 0 after 0 seconds'), 'FontSize', 10, 'Color', [.6 .2 .6]);

% Start timer
tic

% Go through iterations of randomly move UFOs, then move robot. Check for
% hits and update score and timer
while ~isempty(find(0 < ufoFleet.healthRemaining,1))
    ufoFleet.PlotSingleRandomStep();    
    
    % Get the goal joint state
    goalJointState = GetGoalJointState(blasterRobot,ufoFleet);

    % Fix goal pose back to a small step away from the min/max joint limits
    fixIndexMin = goalJointState' < blasterRobot.qlim(:,1);
    goalJointState(fixIndexMin) = blasterRobot.qlim(fixIndexMin,1) + 10*pi/180;
    fixIndexMax = blasterRobot.qlim(:,2) < goalJointState';
    goalJointState(fixIndexMax) = blasterRobot.qlim(fixIndexMax,2) - 10*pi/180;
        
    % Get a trajectory
    jointTrajectory = jtraj(blasterRobot.getpos(),goalJointState,8);
    for armMoveIndex = 1:size(jointTrajectory,1)
        animate(blasterRobot,jointTrajectory(armMoveIndex,:));
        
        endEffectorTr = blasterRobot.fkine(jointTrajectory(armMoveIndex,:));        
        updatedConePoints = [endEffectorTr * [X(:),Y(:),Z(:),ones(numel(X),1)]']';        
        set(cone_h,'XData',reshape(updatedConePoints(:,1),conePointsSize) ...
                  ,'YData',reshape(updatedConePoints(:,2),conePointsSize) ...
                  ,'ZData',reshape(updatedConePoints(:,3),conePointsSize));
              
        coneEnds = [cone_h.XData(2,:)', cone_h.YData(2,:)', cone_h.ZData(2,:)'];        
        ufoHitIndex = CheckIntersections(endEffectorTr,coneEnds,ufoFleet);
        ufoFleet.SetHit(ufoHitIndex);
        currentScore = currentScore + length(ufoHitIndex);
        
        text_h.String = sprintf(['Score: ',num2str(currentScore),' after ',num2str(toc),' seconds']);
        axis([-6,6,-6,6,0,10]);
        % Only plot every 3rd to make it faster
        if mod(armMoveIndex,3) == 0
            drawnow();
        end
    end
    drawnow();
end
display(['Finished in ',num2str(toc),' seconds with score of ',num2str(currentScore)]);
% Log results to file
try load allScoresAndTimes.mat; end
if exist('allScoresAndTimes','var')
    allScoresAndTimes(end+1).seconds = toc;
    allScoresAndTimes(end).score = currentScore;
else
    allScoresAndTimes.seconds = toc;
    allScoresAndTimes.score = currentScore;
end
save('allScoresAndTimes.mat','allScoresAndTimes');

end

%% GetGoalJointState
% Given the current blasterRobot and ufoFleet determine the next pose and
% thus joint state to move to 
function goalJointState = GetGoalJointState(blasterRobot,ufoFleet,solutionToUse)

    if nargin < 3
        solutionToUse = 1;
    end
    
    %% Solution 1: random guess facing upwards
    if solutionToUse == 1
        goalJointState = blasterRobot.getpos() + (rand(1,6)-0.5) * 20*pi/180;
        endEffectorTr = blasterRobot.fkine(goalJointState);

        % Ensure the Z component of the Z axis is positive (pointing upwards),
        % and the Z component of the point is above 1 (approx mid height)
        while endEffectorTr(3,3) < 0.1 || endEffectorTr(3,4) < 1
            goalJointState = blasterRobot.getpos() + (rand(1,6)-0.5) * 20*pi/180;
            endEffectorTr = blasterRobot.fkine(goalJointState);
            display('trying again');
        end
    
    
    %% Solution 2: Randomly select a live member of the ufoFleet to target with ikine
    elseif solutionToUse == 2
        goalJointState = blasterRobot.getpos();
        targetUFOIndex = find(0 < ufoFleet.healthRemaining,1);

        % We want the Z axis of the end-effector to be a line from the
        % end-effector to the ufo base (although we haven't solved for ikine
        % yet)
        actualTr = blasterRobot.fkine(goalJointState);
        aVector =  ufoFleet.model{targetUFOIndex}.base(1:3,4)' - actualTr(1:3,4)';
        % Make a unit vector
        aVector = unit(aVector);
        % What is the angle to the global Z?(if zero use the X and Y axis as
        % is)
        tr = eye(4);
        if ~all(aVector == [0,0,1])
            % N and O axes are arbitary (we don't care about yaw)
            oVector = unit(cross(aVector,[0,0,1]));
            nVector = cross(oVector,aVector);                
            R = [nVector',oVector',aVector'];
            tr(1:3,1:3) = R;
        end
        % Then we don't really care about the x,y,z since the UFOs are far away
        % only that we are point in the correct direction with z axis (so roll
        % and pitch are correct)

        minDistError = sqrt(sum((actualTr(1:3,3)' - aVector).^2));
        bestJointState = goalJointState;
        maxAllowableDistError = 0.2;
        iKineAlpha = 0.9;    

        while maxAllowableDistError < minDistError && 0 < iKineAlpha
            candidateJointState = blasterRobot.ikine(tr,bestJointState,[0,0,0,1,1,0],'alpha',iKineAlpha,'ilimit',20);

            % Fix joint limits if needed
            if ~all(blasterRobot.qlim(:,1) < candidateJointState'  &  candidateJointState' < blasterRobot.qlim(:,2))
                lessThanMinIndex = candidateJointState' < blasterRobot.qlim(:,1);
                greaterThanMaxIndex = blasterRobot.qlim(:,2) < candidateJointState';
                candidateJointState(lessThanMinIndex) = blasterRobot.qlim(lessThanMinIndex,1) + 10*pi/180;
                candidateJointState(greaterThanMaxIndex) = blasterRobot.qlim(greaterThanMaxIndex,2) - 10*pi/180;               
                display('Not in joint limits, so fixing');
            end

            actualTr = blasterRobot.fkine(candidateJointState);
            iKineAlpha = iKineAlpha - 0.1;
            currentDistError = sqrt(sum((actualTr(1:3,3)' - aVector).^2))
            % If less than the best so far and within joint limits
            if  currentDistError < minDistError            
                minDistError = currentDistError;
                bestJointState = candidateJointState;
            end
        end
        % If error is small enough then use it otherwise stay put
        if minDistError < maxAllowableDistError
            goalJointState = bestJointState;
        % If all zero already then choose a random pose
        elseif all(blasterRobot.getpos() == 0)
            goalJointState = (rand(1,6)-0.5) * 30*pi/180;
        % If not already zero and all current solutions are poor then go to zero 
        else
            goalJointState = zeros(1,6);
        end
    end
%     
    %% Implement a better solution here and comment out the above solution
    
    
    
    
end



    