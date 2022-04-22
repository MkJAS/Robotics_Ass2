function PlaceBricks(logFile, robotUR5, brickUR5, robotUR3, brickUR3)
%Function that takes one or two robots/bricks and animates the robot
%collecting and depositing the bricks using the AnimateRobots function
    
    qIntermediaryUR5 = robotUR5.qIntermediary;
        
    %Turn brick positions into poses and make adjust for offsets for UR5
    transformBrickOffsetUR5 = transl(0,0,2*Brick.height) * troty(pi);
    initialBrickPoseUR5 = transl(brickUR5.locationInitial) * transformBrickOffsetUR5;
    finalBrickPoseUR5 = transl(brickUR5.locationFinal) * transformBrickOffsetUR5 * trotz(pi/2);
    brickLocationFinalUR5 = brickUR5.locationFinal;
    
    switch(nargin)
        %Place 2 bricks with robots simultaneously
        case 5
            qIntermediaryUR3 = robotUR3.qIntermediary;

            %Turn brick positions into poses and make adjust for offsets
            %for UR3
            transformBrickOffsetUR3 = transl(0,0,Brick.height) * troty(pi);
            initialBrickPoseUR3 = transl(brickUR3.locationInitial) * transformBrickOffsetUR3;
            finalBrickPoseUR3 = transl(brickUR3.locationFinal) * transformBrickOffsetUR3;
            brickLocationFinalUR3 = brickUR3.locationFinal;

      
            %Go from current robot poses to bricks and delete them
            qCurrentUR3 = robotUR3.model.getpos();
            qCurrentUR5 = robotUR5.model.getpos();
            qTargetUR3 = robotUR3.model.ikcon(initialBrickPoseUR3, qCurrentUR3); %(target pose, current joint angles, mask)
            qTargetUR5 = robotUR5.model.ikcon(initialBrickPoseUR5, qCurrentUR5); %(target pose, current joint angles, mask)
        
            AnimateRobots(logFile, robotUR5, qCurrentUR5, qTargetUR5, robotUR3, qCurrentUR3, qTargetUR3);
            try delete(brickUR3); end
            try delete(brickUR5); end
            
  
            %go from bricks back to intermediary pose
            qCurrentUR3 = robotUR3.model.getpos();
            qCurrentUR5 = robotUR5.model.getpos();
            qTargetUR3 = qIntermediaryUR3;
            qTargetUR5 = qIntermediaryUR5;
        
            AnimateRobots(logFile, robotUR5, qCurrentUR5, qTargetUR5, robotUR3, qCurrentUR3, qTargetUR3);
        
            
            %go from intermediary pose to final brick pose and place 2 bricks
            qCurrentUR3 = robotUR3.model.getpos();
            qCurrentUR5 = robotUR5.model.getpos();
            qTargetUR3 = robotUR3.model.ikcon(finalBrickPoseUR3, qCurrentUR3); %(target pose, current joint angles, mask)
            qTargetUR5 = robotUR5.model.ikcon(finalBrickPoseUR5, qCurrentUR5); %(target pose, current joint angles, mask)
        
            AnimateRobots(logFile, robotUR5, qCurrentUR5, qTargetUR5, robotUR3, qCurrentUR3, qTargetUR3);
            stacked = true;
            Brick(brickLocationFinalUR3, brickLocationFinalUR3, stacked);
            Brick(brickLocationFinalUR5, brickLocationFinalUR5, stacked);
        
            
            %go from final brick position back to intermediary position
            qCurrentUR3 = robotUR3.model.getpos();
            qCurrentUR5 = robotUR5.model.getpos();
            qTargetUR3 = qIntermediaryUR3;
            qTargetUR5 = qIntermediaryUR5;
        
            AnimateRobots(logFile, robotUR5, qCurrentUR5, qTargetUR5, robotUR3, qCurrentUR3, qTargetUR3);
            

        %Only place single brick with single robot 
        case 3 
            %Go from current pose to bricks and delete them
            qCurrentUR5 = robotUR5.model.getpos();
            qTargetUR5 = robotUR5.model.ikcon(initialBrickPoseUR5, qCurrentUR5); %(target pose, current joint angles, mask)
        
            AnimateRobots(logFile, robotUR5, qCurrentUR5, qTargetUR5);
            try delete(brickUR5); end
        
        
            %go from brick back to intermediary pose
            qCurrentUR5 = robotUR5.model.getpos();
            qTargetUR5 = qIntermediaryUR5;
        
            AnimateRobots(logFile, robotUR5, qCurrentUR5, qTargetUR5);
        
            
            %go from intermediary pose to final brick pose and place 2 bricks
            qCurrentUR5 = robotUR5.model.getpos();
            qTargetUR5 = robotUR5.model.ikcon(finalBrickPoseUR5, qCurrentUR5); %(target pose, current joint angles, mask)
        
            AnimateRobots(logFile, robotUR5, qCurrentUR5, qTargetUR5);
            stacked = true;
            Brick(brickLocationFinalUR5, brickLocationFinalUR5, stacked);
        
            
            %go from final brick position back to intermediary position
            qCurrentUR5 = robotUR5.model.getpos();
            qTargetUR5 = qIntermediaryUR5;
        
            AnimateRobots(logFile, robotUR5, qCurrentUR5, qTargetUR5);
            
        %Throw hands.. I mean errors if incorrect args are passed    
        otherwise
            error('incorrect args passed')
    end

   
end

