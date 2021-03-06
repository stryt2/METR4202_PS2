function img = ps2_v2(filename)
%% Initilisation
%clear all;
% close all;

numCylinder = 0;
numCube = 0;
numTriPrism = 0;
figNum = 1;

%% Calibrate Camera
wkspace = evalin('base','whos'); 

if ~ismember('cameraParams',[wkspace(:).name])
    fprintf('CameraParams Calibrating...\n');
    cameraParams = calibrateScript();
    assignin('base', 'cameraParams', cameraParams);
end

%% Read Colourful Image
img_color = imread(filename);
% img_color = imread('wood09.bmp');
% figure(figNum);
% figNum = figNum + 1;
% imshow(img_color);
% title('color');

%% Adjust 
low = 0.2;
high = 1 - low;
gamma = 1 - 2 * low;
in = [low low low; high high high];
out = [low low low; high high high];
gammas = [gamma gamma gamma];
img_adjust = imadjust(img_color,in, out, gammas);
for times = 1:1
    img_adjust = imgaussfilt(img_adjust);
%     img_contrast = medfilt3(img_contrast);
end
% img_contrast = imsharpen(img_contrast);
% figure(figNum);
% figNum = figNum + 1;
% imshow(img_adjust);
% title('adjust');


%% Turn to gray scale
img_gray = rgb2gray(img_adjust);

x = [135 1 1 1280 1280 1175];
y = [1 337 720 720 300 1];
[sz_y, sz_x] = size(img_gray);
boundaryMask = poly2mask(x,y,sz_y,sz_x);
img_gray = img_gray .* uint8(boundaryMask);

% figure(figNum);
% figNum = figNum + 1;
% imshow(img_gray);
% title('gray');

%% Obtain Top Down Transform
if (~exist('fixedPoints') || ~exist('movingPoints'))
    load('cpPoints');
end

tform = cp2tform(movingPoints, fixedPoints, 'projective');
    
img_transform = imtransform(img_gray, tform, 'XYScale', 0.15);

% figure(figNum);
% figNum = figNum + 1;
% imshow(img_transform);
% title('transform');


%%
img_single = im2single(img_color);
rgb_normalised = img_single./repmat(max(sum(img_single, 3), 0.005), [1 1 3]);
% img_gray2 = imadjust(rgb2gray(rgb_normalised));
img_edged0 = edge(imadjust(rgb2gray(rgb_normalised)),'canny', 0.25703);
% figure(figNum);
% figNum = figNum + 1;
% imshow(img_edged0);
% title('edged0');

%% Obtain Edged Image from adjusted gray scale image
% img_edged = edge(imgaussfilt(img_gray),'Prewitt');
img_edged1 = edge(img_gray,'canny', 0.06183);%, 0.08);
% figure(figNum);
% figNum = figNum + 1;
% imshow(img_edged1);
% title('edged1');

%% Obtain Edged Image from original colorful image
% img_edged = edge(imgaussfilt(img_gray),'Prewitt');
img_edged2 = edge(rgb2gray(imgaussfilt(img_color)),'canny', 0.1);%, 0.08);
% figure(figNum);
% figNum = figNum + 1;
% imshow(img_edged2);
% title('edged2');

%% Obtain Edged Image from Shadow Removal
% img_noShadow = shadowRemoval(imgaussfilt(img_color), 15);
img_noShadow = shadowRemoval(img_color, 15);
img_edged3 = edge(img_noShadow,'canny', 0.109);%, 0.08);
% figure(figNum);
% figNum = figNum + 1;
% imshow(img_edged3);
% title('edged3');

%% Combine Edges
se = strel('disk',3);
img_edged0 = img_edged0 .* boundaryMask;
img_edged1 = imdilate(img_edged1,se) .* boundaryMask;
img_edged2 = imdilate(img_edged2,se) .* boundaryMask;
img_edged3 = imdilate(img_edged3,se) .* boundaryMask;
% img_edged = bitor(bitand(bitand(img_edged2, img_edged3), img_edged1), ...
%     img_edged0);
img_edged = bitor(bitand(bitand(img_edged2, img_edged3), img_edged1), ...
    img_edged0);
% figure(figNum);
% figNum = figNum + 1;
% imshow(img_edged);
% title('edgedSum');

%% Dilate Edges
% figure(4);
% dilWidth = 12;
dilWidth = [8 12];
se = strel('disk',dilWidth(1));
img_dilated1 = imdilate(img_edged,se);

se = strel('disk',dilWidth(2));
img_dilated2 = imdilate(img_edged,se);
% imshow(img_dilated);

% se = [strel('line',3,0), strel('line',3,45), strel('line',3,90), ...
%     strel('line',3,135)];
% img_dilated_line = imdilate(img_edged,se);
% imshow(img_dilated_line);

%% Fill Holes
% figure(figNum);
% figNum = figNum + 1;
img_filled1 = imfill(img_dilated1, 'holes');
img_filled2 = imfill(img_dilated2, 'holes');
img_filled1 = bwareaopen(img_filled1,2750);
img_filled2 = bwareaopen(img_filled2,2750);
% imshow(img_filled);

%% Clear Boarders
img_bcleared1 = imclearborder(img_filled1);
img_bcleared2 = imclearborder(img_filled2);
% img_bcleared_line = imclearborder(img_dilated_line);
% figure(figNum);
% figNum = figNum + 1;
% imshow(img_bcleared1);
% title('dilated, filled, bcleared');

%% 
% figure(7);
% img_area = bwpropfilt(img_bcleared, 3000);
% imshow(img_area);

%% Obtain Hough Transform
%close all
% [H, T, R] = hough(img_edged);
% 
% N = 50;
% P  = houghpeaks(H,15,'threshold',ceil(0.04364*max(H(:))));
% 
% lines = houghlines(img_edged,T,R,P,'FillGap',30,'MinLength',20);


% figure(figNum);
% figNum = figNum + 1;
% % subplot(2, 1, 1);
% imshow(img_edged);
% hold on;

% max_len = 0;
% for k = 1:length(lines)
%    xy = [lines(k).point1; lines(k).point2];
%    plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
% 
%    % Plot beginnings and ends of lines
%    plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
%    plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');
% 
%    % Determine the endpoints of the longest line segment
%    len = norm(lines(k).point1 - lines(k).point2);
%    if ( len > max_len)
%       max_len = len;
%       xy_long = xy;
%    end
% end

% title('Hough');

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
% KK = [
%     981.7016    0           637.7308;
%     0           980.4344    380.2695;
%     0           0           1
%     ];
% 
% R = cameraParams.RotationMatrices(:,:,1);
% t = cameraParams.TranslationVectors(1,:);
% R(3,:) = t;
% H = R * cameraParams.IntrinsicMatrix;
% J = imwarp(img_gray, projective2d(inv(H)));
% imshow(J);


%% Trace region boundaries in binary image and Put Box Around Shapes
img_propfilt1 = img_bcleared1;
img_propfilt2 = img_bcleared2;

img_box1 = img_color;

stats1 = regionprops('table', img_propfilt1,'Centroid', 'BoundingBox',...
    'MajorAxisLength','MinorAxisLength', 'Area', 'Extrema');
stats2 = regionprops('table', img_propfilt2,'Centroid', 'BoundingBox',...
    'MajorAxisLength','MinorAxisLength', 'Area', 'Extrema');

[centers1, radii1] = imfindcircles(img_propfilt1, [30, 50]);
[centers2, radii2] = imfindcircles(img_propfilt2, [30, 50], ...
    'ObjectPolarity', 'dark', 'EdgeThreshold', 0.15);
% [centers, radii] = imfindcircles(img_propfilt, [30, 50]);
centers = vertcat(centers1, centers2);
radii = vertcat(radii1, radii2);

% viscircles(centers, radii);

% for n = 0:size(centers)-1
%     img_box1 = insertText(img_box1, centers(n+1,:), char('a'+n));
% end

[B,L] = bwboundaries(img_propfilt2, 'noholes');
% sz = size(B);
% xMax = zeros([1 sz(1,1)]);

% figure(figNum);
% figNum = figNum + 1;
% imshow(label2rgb(L, @jet, [.5 .5 .5]));
% hold on

areaThrsh = [2750 3000];
%     triArea = [12000 14600];
triArea = [12000 18000];
triAreaTolerance = 100;
major2minorThrsh = 2.1;
major2minorCube = 1.3;

for k = 1:height(stats1)
    identified = 0;
    img_zoomIn = img_edged;
    
%     boundary = B{k};
%     plot(boundary(:,2), boundary(:,1), 'w', 'LineWidth', 2);

%     major2minor = stats1.MajorAxisLength(k) / stats1.MinorAxisLength(k)
    major2minor = stats2.MajorAxisLength(k) / stats2.MinorAxisLength(k);
    
    % Bounding Box
    boundingBox(1:2) = stats1.BoundingBox(k,1:2) + dilWidth(1) / 2;
    boundingBox(3:4) = stats1.BoundingBox(k,3:4) - dilWidth(1);
    extrema = stats1.Extrema{k};
    if (stats2.Area(k) > areaThrsh(2) && major2minor < major2minorThrsh)
        img_box1 = insertShape(img_box1,'Rectangle', ...
            boundingBox, 'LineWidth',5);
        img_box1 = insertText(img_box1, stats1.Centroid(k,:), num2str(k));
    else
        continue;
    end
    
    % Cylinder Detection
    tolerance = 10;
    for n = 1:size(centers,1)
        dx = abs(stats1.Centroid(k,1) - centers(n,1));
        dy = abs(stats1.Centroid(k,2) - centers(n,2));
        centroid2circle = sqrt(dx^2 + dy^2);
        if (centroid2circle < radii(n) + tolerance || ...
                (centers(n,1) > boundingBox(1) && ...
                centers(n,1) < boundingBox(1) + boundingBox(3)) && ...
                centers(n,2) > boundingBox(2) && ...
                centers(n,2) < boundingBox(2) + boundingBox(4))
            numCylinder = numCylinder + 1;
            txt = strcat('Cylinder', sprintf('%02d',numCylinder));
%             txtLoc = [stats.Centroid(k,1) - 32, stats.Centroid(k,2) - 12];
%             img_box1 = insertText(img_box1, txtLoc, txt);
            identified = 1;
            break;
        end
    end
    
    % Detection based on area
    triAreaThresh = triArea;
    if (stats2.Centroid(k,2) < 390)
        triAreaThresh = triAreaThresh - (390 - stats2.Centroid(k,2)) * 19;
    else
        triAreaThresh = triAreaThresh + (stats2.Centroid(k,2) - 390) * 12;
    end
    triAreaThresh = triAreaThresh - triAreaTolerance;
    if (~identified)
        if (stats2.Area(k) > triAreaThresh(2))
            numTriPrism = numTriPrism + 1;
            txt = strcat('Tri. Prism', sprintf('%02d',numTriPrism));
%             txtLoc = [stats.Centroid(k,1) - 32, stats.Centroid(k,2) - 12];
%             img_box1 = insertText(img_box1, txtLoc, txt);
            identified = 1;
        elseif (major2minor < major2minorCube)
            numCube = numCube + 1;
            txt = strcat('Cube', sprintf('%02d',numCube));
            identified = 1;
        end
    end
    
    if (~identified) 
        [centers3, radii3] = imfindcircles(img_propfilt1, [30, 60], ...
            'EdgeThreshold', 0.25);
        for n = 1:size(centers3,1)
            dx = abs(stats1.Centroid(k,1) - centers3(n,1));
            dy = abs(stats1.Centroid(k,2) - centers3(n,2));
            centroid2circle = sqrt(dx^2 + dy^2);
            if (centroid2circle < radii3(n) + tolerance + 5)
                numCylinder = numCylinder + 1;
                txt = strcat('Cylinder', sprintf('%02d',numCylinder));
                identified = 1;
                break;
            end
        end
    end
    
    if (identified)
        txtLoc = [stats1.Centroid(k,1) - 32, stats1.Centroid(k,2) - 12];
        img_box1 = insertText(img_box1, txtLoc, txt);
    end
    
%     % Corners Detection
%     temp = round(stats.Extrema{k});
%     temp(:,1) = temp(:,1) - min(temp(:,1)) + 1;
%     temp(:,2) = temp(:,2) - min(temp(:,2)) + 1;
%     mask = zeros(max(temp(:,1)),max(temp(:,2)));
%     for cnt2 = 1:8
%         mask(temp(cnt2,1),temp(cnt2,2))=1;
%     end
%     se = strel('disk',6);
%     mask2 = imdilate(mask,se);
% %     figure(figNum);
% %     figNum = figNum + 1;
% %     imagesc(mask2);
%     [labeled, corners] = bwlabel(mask2,8);
% %     corners
%     if (corners < 6 && ~identified)
%         numTriPrism = numTriPrism + 1;
%         txt = strcat('Tri. Prism', sprintf('%02d',numTriPrism));
%         txtLoc = [stats.Centroid(k,1) - 32, stats.Centroid(k,2) - 12];
%         img_box1 = insertText(img_box1, txtLoc, txt);
%         identified = 1;
%     end
    
    
%     % Mask by Bounding Box
%     x = [boundingBox(1), boundingBox(1), ...
%         boundingBox(1) + boundingBox(3), boundingBox(1) + boundingBox(3)];
%     y = [boundingBox(2), boundingBox(2) + boundingBox(4), ...
%         boundingBox(2) + boundingBox(4), boundingBox(2),];
%     img_zoomIn = img_zoomIn .* poly2mask(x, y, sz_y, sz_x);
%     img_zoomIn = imresize(img_zoomIn, 3);
%     [domino_c, domino_r] = imfindcircles(img_zoomIn, [20, 30], ...
%         'Sensitivity', 0.1);
%     figure(figNum);
%     figNum = figNum + 1;
%     imshow(img_zoomIn);
%     viscircles(domino_c, domino_r);
%     title('zoomed');
%     
%     [H, T, R] = hough(img_zoomIn);
% 
%     N = 50;
%     P  = houghpeaks(H,9,'threshold',ceil(0.04364*max(H(:))));
% 
%     lines = houghlines(img_zoomIn,T,R,P,'FillGap',30,'MinLength',20);
% 
% 
%     figure(figNum);
%     figNum = figNum + 1;
%     imshow(img_zoomIn);
%     hold on;
%     
%     lens = zeros(length(lines), 1);
%     max_len = 0;
%     for line = 1:length(lines)
%         xy = [lines(line).point1; lines(line).point2];
%         plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
% 
%         % Plot beginnings and ends of lines
%         plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
%         plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');
% 
%         % Determine the endpoints of the longest line segment
%         len = norm(lines(line).point1 - lines(line).point2);
%         lens(line) = len;
%         if ( len > max_len)
%             lineNum = line;
%             max_len = len;
%             xy_long = xy;
%         end
%     end
%     hold off
%     
% %     for cnt = 1:max(max(H))
% %         data(cnt) = sum(sum(H == cnt));
% %     end
% %     
% %     [maxval,maxind] = max(data);
% %     medval = median(data);
% % 
% %     [p]=polyfit(1:maxind-5,data(1:maxind-5),2);
% %     
% %     if maxval<3*medval
% % %         set(handles.txtResult,'string','Triangle');
% %         fprintf('tri');
% %     elseif  p(3)>100
% % %         set(handles.txtResult,'string','Square');
% %         fprintf('sq');
% %     else
% % %         set(handles.txtResult,'string','Round'); 
% %         fprintf('ro');
% %     end
    
%     k
end

txtCylinder = ['Total of ' num2str(numCylinder,'%d') ' cylinder(s) found. '];
txtCube = ['Total of ' num2str(numCube,'%d') ' cube(s) found. '];
txtTriPrism = ['Total of ' num2str(numTriPrism,'%d') ' triangular prism(s) found. '];
txtLoc1 = [20, sz_y - 30];
txtLoc2 = [20, sz_y - 60];
txtLoc3 = [20, sz_y - 90];
img_box1 = insertText(img_box1, txtLoc1, txtCylinder);
img_box1 = insertText(img_box1, txtLoc2, txtCube);
img_box1 = insertText(img_box1, txtLoc3, txtTriPrism);

% figure(figNum);
% figNum = figNum + 1;
% imshow(img_box1);

img = img_box1;

% viscircles(centers, radii);
% viscircles(centers3, radii3);
% viscircles(domino_c, domino_r);
end