function LightCurtainLasers %[outputArg1,outputArg2] = untitled(inputArg1,inputArg2)
    %LightCurtainLasers function that plots a line between two points on
    %the lightcurtains to create a light barrier between robot and user
    offsetLightCurtain = 0.4;
    tableHeight = 0.711547;
    offsetVertical = 0.05;

    for index = 1:6
        startP1 = ([-offsetLightCurtain, offsetLightCurtain, tableHeight + offsetVertical]);
        endP1 = ([offsetLightCurtain, offsetLightCurtain, tableHeight + offsetVertical]);
        plot3([startP1(1), endP1(1)], [startP1(2), endP1(2)], [startP1(3), endP1(3)], 'r');

        startP2 = ([offsetLightCurtain, offsetLightCurtain, tableHeight + offsetVertical]);
        endP2 = ([offsetLightCurtain, -offsetLightCurtain, tableHeight + offsetVertical]);
        plot3([startP2(1), endP2(1)], [startP2(2), endP2(2)], [startP2(3), endP2(3)], 'r');

        startP3 = ([offsetLightCurtain, -offsetLightCurtain, tableHeight + offsetVertical]);
        endP3 = ([-offsetLightCurtain, -offsetLightCurtain, tableHeight + offsetVertical]);
        plot3([startP3(1), endP3(1)], [startP3(2), endP3(2)], [startP3(3), endP3(3)], 'r');

        startP4 = ([-offsetLightCurtain, -offsetLightCurtain, tableHeight + offsetVertical]);
        endP4 = ([-offsetLightCurtain, offsetLightCurtain, tableHeight + offsetVertical]);
        plot3([startP4(1), endP4(1)], [startP4(2), endP4(2)], [startP4(3), endP4(3)], 'r');

        offsetVertical = offsetVertical + 0.075;
    end

    % startP = ([-offsetLightCurtain, offsetLightCurtain, tableHeight+0.05]);
    % endP = ([offsetLightCurtain, offsetLightCurtain, tableHeight+0.05]);
    % line1_h = plot3([startP(1), endP(1)], [startP(2), endP(2)], [startP(3), endP(3)], 'r');
    %
    % startP = ([-offsetLightCurtain, offsetLightCurtain, tableHeight+0.15]);
    % endP = ([offsetLightCurtain, offsetLightCurtain, tableHeight+0.15]);
    % line2_h = plot3([startP(1), endP(1)], [startP(2), endP(2)], [startP(3), endP(3)], 'r');
    %
    % startP = ([-offsetLightCurtain, offsetLightCurtain, tableHeight+0.25]);
    % endP = ([offsetLightCurtain, offsetLightCurtain, tableHeight+0.25]);
    % line3_h = plot3([startP(1), endP(1)], [startP(2), endP(2)], [startP(3), endP(3)], 'r');
    %
    % startP = ([-offsetLightCurtain, offsetLightCurtain, tableHeight+0.35]);
    % endP = ([offsetLightCurtain, offsetLightCurtain, tableHeight+0.35]);
    % line4_h = plot3([startP(1), endP(1)], [startP(2), endP(2)], [startP(3), endP(3)], 'r');
    %
    % startP = ([-offsetLightCurtain, offsetLightCurtain, tableHeight+0.45]);
    % endP = ([offsetLightCurtain, offsetLightCurtain, tableHeight+0.45]);
    % line5_h = plot3([startP(1), endP(1)], [startP(2), endP(2)], [startP(3), endP(3)], 'r');
end
