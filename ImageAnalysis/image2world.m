rosshutdown;
rosinit;
sub = rossubscriber('/camera/aligned_depth_to_color/image_raw');
sub2 = rossubscriber('/camera/aligned_depth_to_color/camera_info');
sub3 = rossubscriber('/camera/color/image_raw');
sub4 = rossubscriber('/camera/depth/image_rect_raw');
pause(1);
msg = receive(sub);
msg2 = receive(sub2);
msg3 = receive(sub3);
msg4 = receive(sub4);
rosshutdown;
%%

D = double(msg.readImage);
figure (1)
imshow(I);
I2 = msg3.readImage;
figure (2)
imshow(I2);
%%
K = msg2.K;
point = zeros(480,640,3);
for i=1:640
    for j=1:480
        x = (i - K(3))/K(1);
        y = (j - K(6))/K(5);
        f = 1;
        ux = x*f;
        uy = y*f;
        x = ux;
        y = uy;
        point(j,i,1) = D(j,i) * x;
        point(j,i,2) = D(j,i) * y;
        point(j,i,3) = D(j,i);
    end
end
px = point(:,:,1);
py = point(:,:,2);
pz = point(:,:,3);

% x = (1 - intrin->ppx) / intrin->fx;
% y = (pixel[1] - intrin->ppy) / intrin->fy;
%     if(intrin->model == RS2_DISTORTION_INVERSE_BROWN_CONRADY)
%     {
%         float r2  = x*x + y*y;
%         float f = 1 + intrin->coeffs[0]*r2 + intrin->coeffs[1]*r2*r2 + intrin->coeffs[4]*r2*r2*r2;
%         float ux = x*f + 2*intrin->coeffs[2]*x*y + intrin->coeffs[3]*(r2 + 2*x*x);
%         float uy = y*f + 2*intrin->coeffs[3]*x*y + intrin->coeffs[2]*(r2 + 2*y*y);
%         x = ux;
%         y = uy;
%     }
%     point[0] = depth * x;
%     point[1] = depth * y;
%     point[2] = depth;


