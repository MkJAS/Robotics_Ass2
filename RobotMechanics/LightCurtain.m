function LightCurtain(height)
    %LightCurtainLasers function that plots a line between two points on
    %the lightcurtains to create a light barrier between robot and user
    offsetLightCurtain = 0.4;
    offsetVertical = 0.05;

    PlaceObject('EmergencyButton.ply', [0.5, 0.5, height]);
    PlaceObject('LightCurtain.ply', [-offsetLightCurtain, offsetLightCurtain, height]);
    PlaceObject('LightCurtain.ply', [offsetLightCurtain, offsetLightCurtain, height]);
    PlaceObject('LightCurtainFlipped.ply', [-offsetLightCurtain, -offsetLightCurtain, height]);
    PlaceObject('LightCurtainFlipped.ply', [offsetLightCurtain, -offsetLightCurtain, height]);
    PlaceObject('Lid.ply', [0, 0, (height + 0.52)]);

    % for index = 1:6
    %     startP1 = ([-offsetLightCurtain, offsetLightCurtain, height + offsetVertical]);
    %     endP1 = ([offsetLightCurtain, offsetLightCurtain, height + offsetVertical]);
    %     plot3([startP1(1), endP1(1)], [startP1(2), endP1(2)], [startP1(3), endP1(3)], 'r');

    %     startP2 = ([offsetLightCurtain, offsetLightCurtain, height + offsetVertical]);
    %     endP2 = ([offsetLightCurtain, -offsetLightCurtain, height + offsetVertical]);
    %     plot3([startP2(1), endP2(1)], [startP2(2), endP2(2)], [startP2(3), endP2(3)], 'r');

    %     startP3 = ([offsetLightCurtain, -offsetLightCurtain, height + offsetVertical]);
    %     endP3 = ([-offsetLightCurtain, -offsetLightCurtain, height + offsetVertical]);
    %     plot3([startP3(1), endP3(1)], [startP3(2), endP3(2)], [startP3(3), endP3(3)], 'r');

    %     startP4 = ([-offsetLightCurtain, -offsetLightCurtain, height + offsetVertical]);
    %     endP4 = ([-offsetLightCurtain, offsetLightCurtain, height + offsetVertical]);
    %     plot3([startP4(1), endP4(1)], [startP4(2), endP4(2)], [startP4(3), endP4(3)], 'r');

    %     offsetVertical = offsetVertical + 0.075;
    % end

end
