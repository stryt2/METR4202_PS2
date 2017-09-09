%clear all;
close all;

%% Calibrate Camera
if ~exist('cameraParams')
    cameraParams = calibrateScript();
end

%% Read Colourful Image
figNum = 1;
figure(figNum);
figNum = figNum + 1;
img_color = imread('green01.bmp');
imshow(img_color);
title('color')

%% Adjust 
figure(figNum);
figNum = figNum + 1;
img_adjust = img_color;
for times = 1:1
%     img_contrast = imgaussfilt(img_contrast);
%     img_adjust = medfilt3(img_adjust);
end
% img_contrast = imadjust(img_color, [0.005 1], [0 1]);
%img_contrast = imsharpen(img_contrast);
imshow(img_adjust);
title('contrast')

%% Shadow Removal
figure(figNum);
figNum = figNum + 1;
img_noShadow = img_adjust;
img_noShadow = shadowRemoval(img_noShadow, 15);
imshow(img_noShadow);
title('noShadow')

%% Obtain Edged Image
figure(figNum);
figNum = figNum + 1;
img_edge = medfilt2(img_noShadow);
delta = 0;
thrsh = 0.18;
img_edged2 = edge(img_edge,'Canny', thrsh);