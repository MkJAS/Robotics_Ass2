function getThing(hObject, eventdata, handles,midPoint,endPoint,object,finalLoc,name)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
                qMatrix = RMRC(midPoint, 1, handles.robot);
                for i=1:size(qMatrix,1)
                    handles = guidata(hObject);
                    collision = willCollide(handles.robot, qMatrix(i,:),handles.pcPoints);
                    intruder = lightCurtainCheck(handles.pcPoints);
                    if collision == true
                        printToConsole(1);
                        while(collision==true)
                            handles = guidata(hObject);
                            collision = willCollide(robot, qMatrix(i,:), points);
                            pause(0.01);
                        end
                    elseif intruder == true
                        printToConsole(2);
                        while(intruder==true)
                            handles = guidata(hObject);
                            intruder = lightCurtainCheck(handles.pcPoints);
                            pause(0.01);
                        end
                    else
                        animateRobot(handles.robot,qMatrix(i,:))
                    end 
                end  
                qMatrix = RMRC(endPoint, 2, handles.robot);
                for i=1:size(qMatrix,1)
                    handles = guidata(hObject);
                    collision = willCollide(handles.robot, qMatrix(i,:),handles.pcPoints);
                    intruder = lightCurtainCheck(handles.pcPoints);
                    if collision == true
                        printToConsole(1);
                        while(collision==true)
                            handles = guidata(hObject);
                            collision = willCollide(robot, qMatrix(i,:), points);
                            pause(0.01);
                        end
                    elseif intruder == true
                        printToConsole(2);
                        while(intruder==true)
                            handles = guidata(hObject);
                            intruder = lightCurtainCheck(handles.pcPoints);
                            pause(0.01);
                        end
                    else
                        animateRobot(handles.robot,qMatrix(i,:),object)
                    end 
                end
               PositionObject(handles.robot, finalLoc, name);
 
%             handles.countStrawberry = handles.countStrawberry + 1;
            intPoint = endPoint;
            intPoint(3) = intPoint(3) + 0.1;
            qMatrix = RMRC(intPoint, 1, handles.robot);
            for i=1:size(qMatrix,1)
                handles = guidata(hObject);
                collision = willCollide(handles.robot, qMatrix(i,:),handles.pcPoints);
                intruder = lightCurtainCheck(handles.pcPoints);
                if collision == true
                    printToConsole(1);
                    while(collision==true)
                        handles = guidata(hObject);
                        collision = willCollide(robot, qMatrix(i,:), points);
                        pause(0.01);
                    end 
                elseif  intruder == true
                    printToConsole(2);
                    while(intruder==true)
                        handles = guidata(hObject);
                        intruder = lightCurtainCheck(handles.pcPoints);
                        pause(0.01);
                    end 
                else 
                    animateRobot(handles.robot,qMatrix(i,:))
                end  
            end
end