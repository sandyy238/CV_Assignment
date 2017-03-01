numImages = 14;
files = cell(1,numImages);
for i = 1:numImages
    files{i} = fullfile('/home/sandyy/CV/photos', sprintf('img%d.jpg', i));
end

magnification = 25;
%figure; imshow(files{1}, 'InitialMagnification', magnification);
%title('One of the Calibration Images');

% Detect the checkerboard corners in the images.
[imagePoints, boardSize] = detectCheckerboardPoints(files);

% Generate the world coordinates of the checkerboard corners in the
% pattern-centric coordinate system, with the upper-left corner at (0,0).
squareSize = 23; % in millimeters
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Calibrate the camera.
cameraParams = estimateCameraParameters(imagePoints, worldPoints);

% Evaluate calibration accuracy.
%figure; showReprojectionErrors(cameraParams);
%title('Reprojection Errors');


imOrig = imread(fullfile('/home/sandyy/CV/photos', 'img15.jpg'));
%figure; imshow(imOrig, 'InitialMagnification', magnification);
%title('Input Image');
[im, newOrigin] = undistortImage(imOrig, cameraParams);
%figure; imshow(im, 'InitialMagnification', magnification);
%title('Undistorted Image');

% Convert the image to the HSV color space.
imHSV = rgb2hsv(im);

% Get the saturation channel.
saturation = imHSV(:, :, 2);

% Threshold the image
t = graythresh(saturation);
imCoin = (saturation > t);

figure; imshow(imCoin, 'InitialMagnification', magnification);
title('Segmented Pen');

% Find connected components.
blobAnalysis = vision.BlobAnalysis('AreaOutputPort', true,...
    'CentroidOutputPort', false,...
    'BoundingBoxOutputPort', true,...
    'MinimumBlobArea', 200, 'ExcludeBorderBlobs', true);
[areas, boxes] = step(blobAnalysis, imCoin);

% Sort connected components in descending order by area
[~, idx] = sort(areas, 'Descend');

% Get the two largest components.
boxes = double(boxes(idx(1), :));

% Adjust for coordinate system shift caused by undistortImage
boxes(:,1:2) = bsxfun(@plus, boxes(:,1:2), newOrigin);



% Reduce the size of the image for display.
scale = magnification / 10;
imDetectedCoins = imresize(im, scale);

% Insert labels for the coins.
imDetectedCoins = insertObjectAnnotation(imDetectedCoins, 'rectangle', ...
    scale * boxes, 'Pen');
figure; imshow(imDetectedCoins);
title('Detected Pen');



% Detect the checkerboard.
[imagePoints, boardSize] = detectCheckerboardPoints(im);

% Compute rotation and translation of the camera.
[R, t] = extrinsics(imagePoints, worldPoints, cameraParams);




% Get the top-left and the top-right corners.
box1 = double(boxes(1, :));
imagePoints1 = [box1(1:2); box1(1) + box1(3), box1(2) - box1(4)];
    

% Get the world coordinates of the corners
worldPoints1 = pointsToWorld(cameraParams, R, t, imagePoints1);
dist = sqrt((worldPoints1(1,1) - worldPoints(2,1))^2 + (worldPoints1(1,2)-worldPoints1(2,2))^2);

fprintf('Measured Length of Pen = %0.2f cm\n', dist / 10);