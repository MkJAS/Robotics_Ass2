
% rosshutdown
% rosinit
% sub=rossubscriber("/camera/color/image_raw");
% pause(1);
% [msg2] = receive(sub);
close all
image = msg3.readImage;
% %image = rgb2gray(image);
% % Convert RGB image to chosen color space
I = rgb2hsv(image);

% Define thresholds for channel 1 based on histogram settings
channel1Min(1) = 0.393;
channel1Max(1) = 0.619;

% Define thresholds for channel 2 based on histogram settings
channel2Min(1) = 0.370;
channel2Max(1) = 1.000;

% Define thresholds for channel 3 based on histogram settings
channel3Min(1) = 0.445;
channel3Max(1) = 0.785;

% Define thresholds for channel 1 based on histogram settings
channel1Min(2) = 0.213;
channel1Max(2) = 0.281;

% Define thresholds for channel 2 based on histogram settings
channel2Min(2) = 0.419;
channel2Max(2) = 1.000;

% Define thresholds for channel 3 based on histogram settings
channel3Min(2) = 0.377;
channel3Max(2) = 0.785;

% Define thresholds for channel 1 based on histogram settings
channel1Min(3) = 0.065;
channel1Max(3) = 0.100;

% Define thresholds for channel 2 based on histogram settings
channel2Min(3) = 0.774;
channel2Max(3) = 1.000;

% Define thresholds for channel 3 based on histogram settings
channel3Min(3) = 0.694;
channel3Max(3) = 0.751;

% Define thresholds for channel 1 based on histogram settings
channel1Min(4) = 0.104;
channel1Max(4) = 0.176;

% Define thresholds for channel 2 based on histogram settings
channel2Min(4) = 0.528;
channel2Max(4) = 1.000;

% Define thresholds for channel 3 based on histogram settings
channel3Min(4) = 0.691;
channel3Max(4) = 0.770;


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
    boundary{i} = B{1};
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

K = msg2.K;

% Camera Parameters
 Camera_Focal_LengthX = K(1);
 Camera_Focal_LengthY = K(5);
 Camera_Principle_Point_X = [K(3)];
 Camera_Principle_Point_Y = [K(6)];

 centers = zeros(1,3,size(channel3Max,2));
 Worldcenters = zeros(1,3,size(channel3Max,2));
 depth = 489.5;
 for i=1:size(channel3Max,2)
    
    x = mid(1,2,i) - Camera_Principle_Point_X;
    y = mid(1,1,i) - Camera_Principle_Point_Y;
     
    xReal = (depth*x) / Camera_Focal_LengthX;
    yReal = (depth*y) / Camera_Focal_LengthY;
    centers(1,:,i) = [depth xReal -yReal]
    cp = transl(centers(1,:,i));
    r =  troty(pi/2)*cp;
    r2c = transl(227.5,0,577)*troty(pi/2);
    Worldcenters(1,:,i) = (r(1:3,4) + r2c(1:3,4))';

 end

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




