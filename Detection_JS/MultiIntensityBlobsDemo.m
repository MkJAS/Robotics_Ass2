% Demo to find all/any blobs in a certain intensity range.  I do this for three different ranges.
% Demo by Image Analyst.
% Requires the Image Processing Toolbox (IPT) because it demonstates some functions
% supplied by that toolbox, plus it uses the "coins" demo image supplied with that toolbox.
% If you have the IPT (you can check by typing ver on the command line), you should be able to
% run this demo code simply by copying and pasting this code into a new editor window,
% and then clicking the green "run" triangle on the toolbar.
% Running time = 0.75 seconds.
% Similar Mathworks demos are on the Mathworks.com web site:
% https://www.mathworks.com/products/image.html#iman
% Code written and posted by ImageAnalyst, originally in July 2009.
% Updated March 2022 for MATLAB release R2022a
%------------------------------------------------------------------------------------------------------------------------------------------------------
function MultiIntensityBlobsDemo()
clc;
% clear all;
fprintf('Running %s.m.\n', mfilename('fullpath'));
originalImage = zeros(200,200); % Create image
% Create three portions of the image that have different intensities.
originalImage(30:70, 30:70) = 110;
originalImage(70:170,90:110) = 175;
originalImage(50:100,150:174) = 190;
originalImage(100:125, 20:60) = 241;
originalImage(150:190, 10:80) = 245;
originalImage(130:170, 130:170) = 250;

% Now analyze the image to find blobs in three different intensity ranges.
AnalyzeImage(originalImage, 100, 120);
msgboxw('That is the 110 blob.  Now click OK to find the blob with intensity 175');
AnalyzeImage(originalImage, 170, 201);
msgboxw('That is the 175 blob.  Now click OK to find the blob with intensity 250');
AnalyzeImage(originalImage, 240, 255);
msgbox('Finished running BlobsDemo.m.  Check out the figure window and the command window for the results.');
return;


%------------------------------------------------------------------------------------------------------------------------------------------------------
% This is the main function that analyzes only one image at a time.
function AnalyzeImage(originalImage, lowThreshold, highThreshold)
fontSize = 17;
binaryImage = (originalImage >= lowThreshold) & (originalImage <= highThreshold);
binaryImage = imfill(binaryImage, 'holes');

[labeledImage, numberOfBlobs] = bwlabel(binaryImage, 8);     % Label each blob so can do calculations on it
coloredLabels = label2rgb (labeledImage, 'hsv', 'k', 'shuffle'); % pseudo random color labels

% Display various images.
close all;  % Close any prior figures.
subplot(3,2,1); imagesc(originalImage); colormap(gray(256)); 
title('Original Image', 'FontSize', fontSize);
impixelinfo; % Set it up so user can mouse around over image and see (x,y) and Gray Level or RGB color in status bar in the lower left of the figure.

subplot(3,2,2); imagesc(binaryImage); colormap(gray(256)); 
caption = sprintf('Binary image.  Found %d blobs in the gray level range %d-%d', numberOfBlobs, lowThreshold, highThreshold);
title(caption, 'FontSize', fontSize);
impixelinfo; % Set it up so user can mouse around over image and see (x,y) and Gray Level or RGB color in status bar in the lower left of the figure.

% Save the labeled image as a tif image.
% imwrite(uint16(labeledImage), 'tif_labels.tif');
% tifimage = imread('tif_labels.tif');
% imtool(tifimage, []);  % Use imtool so she can inspect the values.

subplot(3,2,3); 
imagesc(labeledImage); 
caption = sprintf('Labeled image with %d blobs in the gray level range %d-%d', numberOfBlobs, lowThreshold, highThreshold);
title(caption, 'FontSize', fontSize);
impixelinfo; % Set it up so user can mouse around over image and see (x,y) and Gray Level or RGB color in status bar in the lower left of the figure.

subplot(3,2,4); imagesc(coloredLabels); 
title('Pseudo colored labels', 'FontSize', fontSize);
impixelinfo; % Set it up so user can mouse around over image and see (x,y) and Gray Level or RGB color in status bar in the lower left of the figure.

blobMeasurements = regionprops(labeledImage, 'all');   % Get all the blob properties.
numberOfBlobs = size(blobMeasurements, 1);

% Now plot the label number over the labeled image in the middle left.
hold on;
subplot(3,2,3);
for k = 1 : numberOfBlobs
	% Plot the label number since it's not obvious.  Labeled are assigned top to bottom as they are encountered, then left to right.
	xt = blobMeasurements(k).Centroid(1);
	yt = blobMeasurements(k).Centroid(2);
	text(xt, yt, num2str(k), 'FontSize', 17, 'HorizontalAlignment','center','VerticalAlignment','middle', 'Color', 'r', 'FontWeight', 'bold');
end
hold off;

% Now outline each detected blob in the original image in the lower left.
% bwboundaries returns a cell array, where each cell
% contains the row/column coordinates for an object in the image.
% Plot the borders of all the coins on the original
% grayscale image using the coordinates returned by bwboundaries.
subplot(3,2,5); imagesc(originalImage); 
caption = sprintf('%d Blobs are in the gray level range %d-%d.  Shown outlined in green.', numberOfBlobs, lowThreshold, highThreshold);
title(caption, 'FontSize', fontSize);
impixelinfo; % Set it up so user can mouse around over image and see (x,y) and Gray Level or RGB color in status bar in the lower left of the figure.
hold on;
boundaries = bwboundaries(binaryImage);
for k = 1 : numberOfBlobs
	% Plot the outline.
	thisBoundary = boundaries{k};
	plot(thisBoundary(:,2), thisBoundary(:,1), 'g', 'LineWidth', 2);
end
hold off;

% Now maximize the figure window.
g = gcf;
g.WindowState = 'maximized';

fprintf(1,'Blob #      Mean Intensity  Area     Perimeter  Centroid\n');
for k = 1 : numberOfBlobs           % Loop through all blobs.
	% Find the mean of each blob.  (R2008a has a better way where you can pass the original image
	% directly into regionprops.  The way below works for all versions including earlier versions.)
	thisBlobsPixels = blobMeasurements(k).PixelIdxList;  % Get list of pixels in current blob.
	meanGL = mean(originalImage(thisBlobsPixels));             % Find mean intensity (in original image!)
	blobArea = blobMeasurements(k).Area;		% Get area.
	blobPerimeter = blobMeasurements(k).Perimeter;		% Get perimeter.
	blobCentroid = blobMeasurements(k).Centroid;		% Get centroid.
	fprintf(1,'#%d %18.1f %11.1f %8.1f %8.1f %8.1f\n', k, meanGL, blobArea, blobPerimeter, blobCentroid);
end
return;

%------------------------------------------------------------------------------------------------------------------------------------------------------
function msgboxw(message)
uiwait(msgbox(message));
return;

