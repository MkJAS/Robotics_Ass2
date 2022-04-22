clear all
close all

rosbag info TopDown.bag

bag = rosbag('TopDown.bag');

rgb = select(bag,'Topic','/device_0/sensor_1/Color_0/image/data');
depth = select(bag,'Topic','/device_0/sensor_0/Depth_0/image/data');
camera_info = select(bag,'Topic','/device_0/sensor_1/Color_0/info/camera_info');
D_camera_info = select(bag,'Topic','/device_0/sensor_0/Depth_0/info/camera_info');




Image_msg = readMessages(rgb,1);
Depth_msg = readMessages(depth,1);
camera_info_msg = readMessages(camera_info,1);
D_camera_info_msg = readMessages(D_camera_info,1);


Y = typecast(Depth_msg{1,1}.Data,'uint16');
count = 1;
V = zeros(480,848);
for i=1:480
    for j=1:848
        V(i,j) = Y(count);
         count = count + 1;
    end    
end
rgbData = Image_msg{1,1}.readImage;
depthData = V;
clear V;






%%

% depthLim =5000; %remove all depth data greater than depthLim
% fn = ('TopDown.bag');
% 
% %image data stored in the following
% dataType = {'/device_0/sensor_0/Depth_0/image/data';...
%         '/device_0/sensor_1/Color_0/image/data'};
% 
% 
% for i=1:2
%     dataT=dataType{i};
% %     bag = rosbag(fn);
%     bagselect1 = select(bag, 'Topic', dataT)
%     msgs = readMessages(bagselect1);
%     msgs{1};
%     [img,alpha] = readImage(msgs{1});
%     try
%         figure(1)
%         imshow(img,[min(min(img)) max(max(img))/10]);
%         colormap(jet)
%         img=double(img);
%         img(img>depthLim)=0; 
%         img(img==0)=NaN; % get rid of spurious daa from mesh plot
%         figure(2); 
%         mesh(img)
%         depth = img;
%     catch
%         figure(3)
%         rgb = img;
%         imshow(img)
%     end
%     clear img
%     
% end

    K = camera_info_msg{1,1}.K;
    intrinsics = cameraIntrinsics([K(1) K(5)],[K(3) K(6)],[double(camera_info_msg{1,1}.Height) double(camera_info_msg{1,1}.Width)])
    Kd = D_camera_info_msg{1,1}.K;
    Dintrinsics = cameraIntrinsics([Kd(1) Kd(5)],[Kd(3) Kd(6)],[double(D_camera_info_msg{1,1}.Height) double(D_camera_info_msg{1,1}.Width)])

    depthHeight = Dintrinsics.ImageSize(1);
    depthWidth = Dintrinsics.ImageSize(2);
    rgbHeight = intrinsics.ImageSize(1);
    rgbWidth = intrinsics.ImageSize(2);
    
    
    depthScale = 1;
    fx_d = Dintrinsics.FocalLength(1);
    fy_d = Dintrinsics.FocalLength(2);
    cx_d = Dintrinsics.PrincipalPoint(1);
    cy_d = Dintrinsics.PrincipalPoint(2);
    
    extrinsic = transl(0.0151737350970507,1.76809862750815e-05,-0.000113747271825559)*1000;
    extrinsic(1:3,1:3) = [0.99996,0.00565148,0.00696151;
  -0.00566533,0.999982,0.00197255;
  -0.00695024,-0.00201191,0.999974];
    
    fx_rgb = intrinsics.FocalLength(1);
    fy_rgb = intrinsics.FocalLength(2);
    cx_rgb = intrinsics.PrincipalPoint(1);
    cy_rgb = intrinsics.PrincipalPoint(2);
    % Aligned will contain X, Y, Z, R, G, B values in its planes
    aligned = zeros(depthHeight, depthWidth, 6);

    for v = 1 : (depthHeight)
        for u = 1 : (depthWidth)
            % Apply depth intrinsics
            z = single(depthData(v,u)) / depthScale;
            x = single((u - cx_d) * z) / fx_d;
            y = single((v - cy_d) * z) / fy_d;
            
            % Apply the extrinsics
            transformed = (extrinsic * [x;y;z;1])';
            aligned(v,u,1) = transformed(1);
            aligned(v,u,2) = transformed(2);
            aligned(v,u,3) = transformed(3);
        end
    end

    for v = 1 : (depthHeight)
        for u = 1 : (depthWidth)
            % Apply RGB intrinsics
            x = (aligned(v,u,1) * fx_rgb / aligned(v,u,3)) + cx_rgb;
            y = (aligned(v,u,2) * fy_rgb / aligned(v,u,3)) + cy_rgb;
            
            % "x" and "y" are indices into the RGB frame, but they may contain
            % invalid values (which correspond to the parts of the scene not visible
            % to the RGB camera.
            % Do we have a valid index?
            if (x > rgbWidth || y > rgbHeight ||...
                x < 1 || y < 1 ||...
                isnan(x) || isnan(y))
                continue;
            end
            
            % Need some kind of interpolation. I just did it the lazy way
            x = round(x);
            y = round(y);

            aligned(v,u,4) = single(rgbData(y, x, 1));
            aligned(v,u,5) = single(rgbData(y, x, 2));
            aligned(v,u,6) = single(rgbData(y, x, 3));
        end
    end    
    rgb_aligned = uint8(aligned(:,:,4:6));
    Zdepth_aligned = aligned(:,:,3);
    figure (4) 
    subplot(2,1,1)
    imshow(rgb_aligned);
    title('RGB aligned');
    subplot(2,1,2);
    imshow(Zdepth_aligned);
    title('Depth');
%%
count = 1;
V = zeros(480,848);
for i=1:480
    for j=1:848
        V(i,j) = Y(count);
         count = count + 1;
    end    
end
