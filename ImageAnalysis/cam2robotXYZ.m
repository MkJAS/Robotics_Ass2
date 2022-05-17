
% rosshutdown
% rosinit
% % sub=rossubscriber("/camera/color/image_raw");
% pause(1);
% % [msg2] = receive(sub);
% % close all
% % image = msg2.readImage;
% % % image = rgb2gray(image);
% % % % Convert RGB image to chosen color space
% sub=rossubscriber("/camera/color/camera_info");
% [msg2] = receive(sub);

%%
load("image.mat");
I = rgb2hsv(image);
figure (2)
imshow(image);

% Define thresholds for channel 1 based on histogram settings
channel1Min(1) = 0.422;
channel1Max(1) = 0.504;

% Define thresholds for channel 2 based on histogram settings
channel2Min(1) = 0.258;
channel2Max(1) = 1.000;

% Define thresholds for channel 3 based on histogram settings
channel3Min(1) = 0.000;
channel3Max(1) = 1.000;

% Define thresholds for channel 1 based on histogram settings
channel1Min(2) = 0.263;
channel1Max(2) = 0.424;

% Define thresholds for channel 2 based on histogram settings
channel2Min(2) = 0.165;
channel2Max(2) = 0.543;

% Define thresholds for channel 3 based on histogram settings
channel3Min(2) = 0.367;
channel3Max(2) = 0.590;

% Define thresholds for channel 1 based on histogram settings
channel1Min(3) = 0.099;
channel1Max(3) = 0.184;

% Define thresholds for channel 2 based on histogram settings
channel2Min(3) = 0.495;
channel2Max(3) = 1.000;

% Define thresholds for channel 3 based on histogram settings
channel3Min(3) = 0.574;
channel3Max(3) = 0.697;

% Define thresholds for channel 1 based on histogram settings
channel1Min(4) = 0.044;
channel1Max(4) = 0.088;

% Define thresholds for channel 2 based on histogram settings
channel2Min(4) = 0.564;
channel2Max(4) = 1.000;

% Define thresholds for channel 3 based on histogram settings
channel3Min(4) = 0.516;
channel3Max(4) = 0.713;



boundary = cell(1,4);
mid = zeros(1,2,4);
for i=1:size(channel3Max,2)
    % Create mask based on chosen histogram thresholds
    sliderBW = (I(:,:,1) >= channel1Min(i) ) & (I(:,:,1) <= channel1Max(i)) & ...
        (I(:,:,2) >= channel2Min(i) ) & (I(:,:,2) <= channel2Max(i)) & ...
        (I(:,:,3) >= channel3Min(i) ) & (I(:,:,3) <= channel3Max(i));
    BW = sliderBW;
    
    % Set background pixels where BW is false to zero.
    maskedRGBImage(repmat(~BW,[1 1 3])) = 0;
    
    [B,L] = bwboundaries(BW,'noholes');
%     figure (1)
%     imshow(label2rgb(L, @jet, [.5 .5 .5]))
    %hold on
    % for k = 1:length(B)
    if i==1
        boundary{i} = B{5};
    else
        boundary{i} = B{1};
    end
    mid(1,:,i) = mean(boundary{i});
end

   figure (1)
   plot(boundary{1}(:,2), boundary{1}(:,1), 'b', 'LineWidth', 2)
   axis([0 640 0 480]);
   set(gca,'YDir','reverse')
   hold on
   scatter(mid(1,2,1),mid(1,1,1))
   plot(boundary{2}(:,2), boundary{2}(:,1), 'g', 'LineWidth', 2)
   plot(boundary{3}(:,2), boundary{3}(:,1), 'o', 'LineWidth', 2)
   plot(boundary{4}(:,2), boundary{4}(:,1), 'y', 'LineWidth', 2)
   scatter(mid(1,2,2),mid(1,1,2))
   scatter(mid(1,2,3),mid(1,1,3))
   scatter(mid(1,2,4),mid(1,1,4))

   
% end
% plot(boundary(:,1), boundary(:,2));

% mid = mean(boundary);%((min(boundary(:,1))) + (max(boundary(:,1))))/2;
% % yb = ((min(boundary(:,2))) + (max(boundary(:,2))))/2;
% hold on;
% scatter(mid(2),mid(1))
% hold off;

%% Calculate 3D Point Coordinates on Camera Frame

K = [608.3387 0 327.5015 0 608.1425 232.3930 0 0 1];

% Camera Parameters
 Camera_Focal_LengthX = K(1);
 Camera_Focal_LengthY = K(5);
 Camera_Principle_Point_X = [K(3)];
 Camera_Principle_Point_Y = [K(6)];

 camCentersReal = zeros(1,3,size(channel3Max,2));
 Worldcenters = zeros(1,3,size(channel3Max,2));
 depth = 592.5;
 for i=1:size(channel3Max,2)
    
    x = mid(1,2,i) - Camera_Principle_Point_X;
    y = mid(1,1,i) - Camera_Principle_Point_Y;
     
    xCam = (depth*x) / Camera_Focal_LengthX;
    yCam = (depth*y) / Camera_Focal_LengthY;
    camCentersReal(1,:,i) = [xCam -yCam   -depth]
    %rotate camera view 180 degrees ---- cameraX = RobotY  cameraY = -RobotX
    axisAlignedRobot = [yCam xCam -depth];
    cp = transl(axisAlignedRobot);
    r2c = transl(335.5,0,528); %transform from robot to camera
    t = r2c*cp;
    Worldcenters(1,:,i) = t(1:3,4)';

 end
 basket = Worldcenters(:,:,1);
 basket(3) = basket(3) + 155; %raise basket coord cause wwant to be above it not at its centre on the ground
 basket
 green = Worldcenters(:,:,2)
 orange = Worldcenters(:,:,3)
 yellow = Worldcenters(:,:,4)

%         x = (i - K(3))/K(1);
%         y = (j - K(6))/K(5);

% % Move origin to principle point:
% %     % RED Cube
% %         Red_X_L = xr - Camera_Principle_Point_X;
% %         Red_Y_L = yr - Camera_Principle_Point_Y;
%     % BLUE Cube
%         Blue_X_L = mid(2) - Camera_Principle_Point_X;
%         Blue_Y_L = mid(1) - Camera_Principle_Point_Y;
% 
% % Collect Depth (Z value) information from Camera (ROS-SUBSCRIBE???? OR MEASURE)
% %     Red_Z_Camera = 489.5;       %400mm from camera to base
%     depth = 489.5;      
% 
% % Calculate X and Y Coordinates using already known Z
% %     Red_X_Camera = (Red_Z_Camera*Red_X_L) / Camera_Focal_Length;
% %     Red_Y_Camera = (Red_Z_Camera*Red_Y_L) / Camera_Focal_Length;
% 
%     xReal = (depth*Blue_X_L) / Camera_Focal_LengthX;
%     xReal = (depth*Blue_Y_L) / Camera_Focal_LengthY;
% 
% % Store in 1 by 3 Matrix of Coordinates (x,y,z)
% %     RED_CamCoordinate = [Red_X_Camera Red_Y_Camera Red_Z_Camera]
%     Blue_CamCoordinate = [xReal xReal depth]

% cp = transl(centers(1,:,1));
% r =  troty(pi/2)*cp;
% r2c = transl(0.2275,0,0.577)*troty(pi/2);
% xyz_World = r(1:3,4) + r2c(1:3,4);

figure (2)
hold on
   scatter(mid(1,2,2),mid(1,1,2),'d','r')
   scatter(mid(1,2,3),mid(1,1,3),'d','r')
   scatter(mid(1,2,4),mid(1,1,4),'d','r')
   scatter(mid(1,2,1),mid(1,1,1),'d','r')
    text(mid(1,2,1),mid(1,1,1),num2str(camCentersReal(:,1:2,1)),'Color','k','LineWidth',3);
    text(mid(1,2,2),mid(1,1,2),num2str(camCentersReal(:,1:2,2)),'Color','k','LineWidth',3);
    text(mid(1,2,3),mid(1,1,3),num2str(camCentersReal(:,1:2,3)),'Color','k','LineWidth',3);
    text(mid(1,2,4),mid(1,1,4),num2str(camCentersReal(:,1:2,4)),'Color','k','LineWidth',3);




