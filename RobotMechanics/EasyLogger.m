function EasyLogger(logFile, robot, qCurrent, qTarget)
%Function that makes it easy for me to log and display transforms
    scriptName = 'in func AnimateRobots';
    transformCurrent = robot.model.fkine(qCurrent);
    transformTarget = robot.model.fkine(qTarget);
    name = robot.name;
    logFile.mlog = {logFile.DEBUG, scriptName, name};
    logFile.mlog = {logFile.DEBUG, scriptName, ['Current transform is:', logFile.MatrixToString(transformCurrent)]};
    logFile.mlog = {logFile.DEBUG, scriptName, ['Target transform is: ', logFile.MatrixToString(transformTarget)]};
    disp([newline,name]);
    disp(strcat('Current transform is: ',logFile.MatrixToString(transformCurrent)));
    disp(strcat('Target transform is:  ',logFile.MatrixToString(transformTarget)))
end

