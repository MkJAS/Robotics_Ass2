function MoveObject(logFile, robot, object)
    %Function that takes one or two robots/objects and animates the robot
    %collecting and depositing the objects using the AnimateRobots function

    qIntermediary = robot.qIntermediary;

    %Turn object positions into poses and make adjust for offsets for
    exampleOffset = transl(0, 0, 2 * object.height) * troty(pi);
    examplePose = transl(object.locationInitial) * exampleOffset;
    exampleFinalPose = transl(object.locationFinal) * exampleOffset * trotz(pi / 2);

    %Go from current pose to objects and delete them
    qCurrent = robot.model.getpos();
    qTarget = robot.model.ikcon(examplePose, qCurrent); %(target pose, current joint angles, mask)

    AnimateRobots(logFile, robot, qCurrent, qTarget);
    try delete(object); end

    %go from object back to intermediary pose
    qCurrent = robot.model.getpos();
    qTarget = qIntermediary;

    AnimateRobots(logFile, robot, qCurrent, qTarget);

    %go from intermediary pose to final object pose and place 2 objects
    qCurrent = robot.model.getpos();
    qTarget = robot.model.ikcon(exampleFinalPose, qCurrent); %(target pose, current joint angles, mask)

    AnimateRobots(logFile, robot, qCurrent, qTarget);
    object(objectLocationFinal, objectLocationFinal, stacked);

    %go from final object position back to intermediary position
    qCurrent = robot.model.getpos();
    qTarget = qIntermediary;

    AnimateRobots(logFile, robot, qCurrent, qTarget);

end
