function MoveFruit(logFile, robot, fruit, locationFinal)
    %Function that takes a robot and a peice of fruit and animates the robot
    %collecting and depositing the fruit using the AnimateRobots function

    qIntermediary = robot.qIntermediary;

    %*Turn fruit positions into poses and make adjust for offsets for
    offsetFruit = transl(0, 0, fruit.height / 2) * troty(pi);
    fruitPose = transl(fruit.location) * offsetFruit;
    fruitFinalPose = transl(locationFinal) * offsetFruit;

    %*Go from current pose to fruit and delete it
    qCurrent = robot.model.getpos();
    qTarget = robot.model.ikcon(fruitPose, qCurrent); %(target pose, current joint angles, mask)

    AnimateRobots(logFile, robot, qCurrent, qTarget);
    try delete(fruit); end

    %*go from fruit back to intermediary pose
    qCurrent = robot.model.getpos();
    qTarget = qIntermediary;

    AnimateRobots(logFile, robot, qCurrent, qTarget);

    %*go from intermediary pose to final fruit pose and place a piece of fruit
    qCurrent = robot.model.getpos();
    qTarget = robot.model.ikcon(fruitFinalPose, qCurrent); %(target pose, current joint angles, mask)

    AnimateRobots(logFile, robot, qCurrent, qTarget);

    if fruit.type == "grape" %todo may need to adjust so there is handle for placed fruit - ie update the current fruits location and have it re-draw itself
        Grape(locationFinal);
    else
        Strawberry(locationFinal);
    end

    %*go from final fruit position back to intermediary position
    qCurrent = robot.model.getpos();
    qTarget = qIntermediary;

    AnimateRobots(logFile, robot, qCurrent, qTarget);
end
