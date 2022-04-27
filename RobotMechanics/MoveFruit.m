function MoveFruit(logFile, robot, fruit, locationFinal)
    %Function that takes a robot and a peice of fruit and animates the robot
    %collecting and depositing the fruit using the AnimateRobots function

    qIntermediary = robot.qIntermediary;

    %*Turn fruit positions into poses and make adjust for offsets for
    fruitOffset = transl(0, 0, fruit.height / 2) * troty(pi);
    fruitPose = transl(fruit.location) * fruitOffset;
    fruitFinalPose = transl(locationFinal) * fruitOffset;

    %*Go from current pose to fruit and delete it
    qCurrent = robot.model.getpos()
    qTarget = robot.model.ikcon(fruitPose, qCurrent); %(target pose, current joint angles, mask)

    AnimateRobots(logFile, robot, qCurrent, qTarget);
    try delete(fruit); end

    %*go from fruit back to intermediary pose
    qCurrent = robot.model.getpos()
    qTarget = qIntermediary;

    AnimateRobots(logFile, robot, qCurrent, qTarget);

    %*go from intermediary pose to final fruit pose and place a piece of fruit
    qCurrent = robot.model.getpos()
    qTarget = robot.model.ikcon(fruitFinalPose, qCurrent); %(target pose, current joint angles, mask)

    AnimateRobots(logFile, robot, qCurrent, qTarget);
    Fruit(locationFinal); % todo may need to adjust this so there is a handle for placed fruit

    %*go from final fruit position back to intermediary position
    qCurrent = robot.model.getpos()
    qTarget = qIntermediary;

    AnimateRobots(logFile, robot, qCurrent, qTarget);
end
