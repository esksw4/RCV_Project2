clc;
close all;
% Define images to process
imageFileNames = {'/Users/e.kim4/Documents/MATLAB/IMG_0152.png',... '/Users/e.kim4/Documents/MATLAB/IMG_0153.png',... '/Users/e.kim4/Documents/MATLAB/IMG_0154.png',... '/Users/e.kim4/Documents/MATLAB/IMG_0155.png',... '/Users/e.kim4/Documents/MATLAB/IMG_0156.png',...
};
% Detect checkerboards in images
[imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(imageFileNames); imageFileNames = imageFileNames(imagesUsed);
% Generate world coordinates of the corners of the squares squareSize = 25; % in units of 'mm'
worldPoints = generateCheckerboardPoints(boardSize, squareSize);
% Calibrate the camera
[cameraParams, imagesUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
'EstimateSkew', false, 'EstimateTangentialDistortion', false, ...
 
'NumRadialDistortionCoefficients', 2, 'WorldUnits', 'mm', ... 'InitialIntrinsicMatrix', [], 'InitialRadialDistortion', []);
% View reprojection errors
figure(1);
subplot(2,2,1); showReprojectionErrors(cameraParams, 'barGraph');
% Visualize pattern locations
subplot(2,2,2);
showExtrinsics(cameraParams, 'CameraCentric'); subplot(2,2,3);
showExtrinsics(cameraParams, 'patternCentric');
% Display parameter estimation errors
displayErrors(estimationErrors, cameraParams);
% For example, you can use the calibration data to remove effects of lens distortion.
originalImage = imread(imageFileNames{1});
undistortedImage = undistortImage(originalImage, cameraParams);