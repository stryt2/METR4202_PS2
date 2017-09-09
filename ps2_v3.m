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
img_edged = edge(img_edge,'Canny', thrsh);
imshow(img_edged);
title('edged');

%% Dilate Edges
figure(figNum);
figNum = figNum + 1;
se = strel('disk',4);
img_dilated = imdilate(img_edged,se);
imshow(img_dilated);
title('dilated');

se = [strel('line',3,0), strel('line',3,45), strel('line',3,90), ...
    strel('line',3,135)];
img_dilated_line = imdilate(img_edged,se);
% imshow(img_dilated_line);

%% Fill Holes
figure(figNum);
figNum = figNum + 1;
img_filled = imfill(img_dilated, 'holes');
img_dilated = bwareaopen(img_filled,1000);
imshow(img_filled);
title('filled');

%% Clear Boarders
figure(figNum);
figNum = figNum + 1;
img_bcleared = imclearborder(img_filled);
img_dilated_line = imclearborder(img_dilated_line);
img_dilated = imclearborder(img_filled);

imshow(img_bcleared);

%% 
% figure(7);
% img_area = bwpropfilt(img_bcleared, 3000);
% imshow(img_area);

%% Obtain Hough Transform
%close all
[H, T, R] = hough(img_dilated);

N = 50;
P  = houghpeaks(H,15,'threshold',ceil(0.04364*max(H(:))));

lines = houghlines(img_dilated,T,R,P,'FillGap',30,'MinLength',20);


figure(figNum);
figNum = figNum + 1;
% subplot(2, 1, 1);
imshow(img_dilated);
hold on;

max_len = 0;
for k = 1:length(lines)
   xy = [lines(k).point1; lines(k).point2];
   plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');

   % Plot beginnings and ends of lines
   plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
   plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');

   % Determine the endpoints of the longest line segment
   len = norm(lines(k).point1 - lines(k).point2);
   if ( len > max_len)
      max_len = len;
      xy_long = xy;
   end
end

% subplot(2, 1, 2);
% imshow(imadjust(mat2gray(H)), 'XData', T, 'YData', R, ...
%     'InitialMagnification', 'fit');
% axis on;
% axis normal;
% grid on;
% hold on;
% colormap('jet');

%% Calibration Matrix
% cpselect

% figure(figNum);
% figNum = figNum + 1;
KK = [
    981.7016    0           637.7308;
    0           980.4344    380.2695;
    0           0           1
    ];

R = cameraParams.RotationMatrices(:,:,1);
t = cameraParams.TranslationVectors(1,:);
R(3,:) = t;
H = R * cameraParams.IntrinsicMatrix;
J = imwarp(img_gray, projective2d(inv(H)));
% imshow(J);


%%

% [C, R] = imfindcircles(img_area, [30, 50]);
% viscircles(C, R);

%% Trace and draw region boundaries in binary image

figure(figNum);
figNum = figNum + 1;
% img_propfilt = bwareaopen(img_dilated, 1000);
[B,L] = bwboundaries(img_dilated, 'noholes');
imshow(label2rgb(L, @jet, [.5 .5 .5]))

sz = size(B);
xMax = zeros([1 sz(1,1)]);
img_box1 = img_color;

stats = regionprops('table', img_dilated,'Centroid', 'BoundingBox',...
    'MajorAxisLength','MinorAxisLength', 'Orientation', 'FilledArea')

hold on
for k = 1:length(B)
   boundary = B{k};
   plot(boundary(:,2), boundary(:,1), 'w', 'LineWidth', 2)

   xmax = max(boundary(:,2));
   xmin = min(boundary(:,2));
   ymax = max(boundary(:,1));
   ymin = min(boundary(:,1));
   
   xMax(k) = xmax;
   xMin(k) = xmin;
   yMax(k) = ymax;
   yMin(k) = ymin;
   
   adjust = 0;
   if (xmax - xmin > 50 && ymax - ymin > 50)
       img_box1 = insertShape(img_box1,'Rectangle',...
           stats.BoundingBox(k,:), 'LineWidth',5);
%        img_box1 = insertShape(img_box1,'Rectangle',...
%            [xmin+adjust ymin+adjust xmax-xmin-adjust ymax-ymin-adjust],...
%            'LineWidth',5);
       img_box1 = insertText(img_box1, stats.Centroid(k,:), num2str(k));
   end
end

figure(figNum);
figNum = figNum + 1;
imshow(img_box1)