% function img = objDetect(filename)
%% Initilisation
%clear all;
% close all;

numCylinder = 0;
numCube = 0;
numTriPrism = 0;
figNum = 1;
nonCollide = 0;

%% Calibrate Camera
% wkspace = evalin('base','whos'); 
% 
% if ~ismember('cameraParams',[wkspace(:).name])
%     fprintf('CameraParams Calibrating...\n');
%     cameraParams = calibrateScript();
%     assignin('base', 'cameraParams', cameraParams);
% end

%% Read Colourful Image

% filename = '1. Basic Environment/wood03.bmp';
filename = '2. Skillful Environment/green05.bmp';

if (ischar(filename))
    img_color = imread(filename);
else
    img_color = uint8(filename);
%     img_color = uint8(videoFrame);
end
% img_color = imread('wood09.bmp');
figure(figNum);
figNum = figNum + 1;
imshow(img_color);
title('color');

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
areaLowThrsh = [6000 8000];
areaHighThrsh = [20000 26700];
img_filled1 = bwareaopen(img_filled1,areaLowThrsh(1));
img_filled2 = bwareaopen(img_filled2,areaLowThrsh(2));
% imshow(img_filled);

%% Clear Boarders
img_bcleared1 = imclearborder(img_filled1);
img_bcleared2 = imclearborder(img_filled2);
% img_bcleared_line = imclearborder(img_dilated_line);
figure(figNum);
figNum = figNum + 1;
imshow(img_bcleared2);
title('dilated, filled, bcleared');


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
barWidth = 20;

img_propfilt1 = img_bcleared1;
img_propfilt2 = img_bcleared2;

img_result = img_color;

stats1 = regionprops('table', img_propfilt1,'Centroid', 'BoundingBox',...
    'MajorAxisLength','MinorAxisLength', 'Area', 'Extrema', 'PixelList')%, ...
    %'ConvexImage')
stats2 = regionprops('table', img_propfilt2,'Centroid', 'BoundingBox',...
    'MajorAxisLength','MinorAxisLength', 'Area', 'Extrema', 'PixelList')%, ...
    %'ConvexImage')

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



%     triArea = [12000 14600];
triAreaBase = [12000 18000];
triAreaTolerance = 100;
major2minorThrsh = 2.1;
major2minorCube = 1.3;

stats = stats2;
statsNum = 2;
img_propfilt = img_propfilt2;
if (height(stats1) > height(stats2))
    stats = stats1;
    statsNum = 1;
    img_propfilt = img_propfilt1;
end

% [B,L] = bwboundaries(img_edged, 'noholes');
[B,L] = bwboundaries(img_propfilt, 'noholes');
% sz = size(B);
% xMax = zeros([1 sz(1,1)]);

figure(figNum);
figNum = figNum + 1;
imshow(label2rgb(L, @jet, [.5 .5 .5]));
hold on

startCoord = [sz_x 0];
endCoord = zeros(1, 2);
startEndObjNum = zeros(1, 2);

stats_major2minor = stats.MajorAxisLength ./ stats.MinorAxisLength;

for k = 3:3%height(stats)
    fprintf('------------------------------------------------------\n');
    k
    identified = 0;
    classficaiton = 0;      % 0: Unidentified, 1: Cylinder, 2: Tri. Prism
                            % 3: Cube
    img_zoomIn = img_edged;
    img_zoomIn_gray = img_gray;
    
    boundary = B{k};
    plot(boundary(:,2), boundary(:,1), 'w', 'LineWidth', 2);

%     major2minor = stats1.MajorAxisLength(k) / stats1.MinorAxisLength(k)
    major2minor = stats_major2minor(k);
    
    % Bounding Box
    boundingBox(1:2) = stats.BoundingBox(k,1:2) + dilWidth(statsNum) / 2;
    boundingBox(3:4) = stats.BoundingBox(k,3:4) - dilWidth(statsNum);
    extrema = stats.Extrema{k};
    
    % Mask by Bounding Box
    x = [boundingBox(1), boundingBox(1), ...
        boundingBox(1) + boundingBox(3), boundingBox(1) + boundingBox(3)];
    y = [boundingBox(2), boundingBox(2) + boundingBox(4), ...
        boundingBox(2) + boundingBox(4), boundingBox(2),];
    img_zoomIn = img_zoomIn .* poly2mask(x, y, sz_y, sz_x);
    figure(figNum);
    figNum = figNum + 1;
    imshow(img_zoomIn);
    title('zoomed');
    
    [H, T, R] = hough(img_zoomIn);

    N = 50;
    P  = houghpeaks(H,9,'threshold',ceil(0.048*max(H(:))));

    lines = houghlines(img_zoomIn,T,R,P,'FillGap',30,'MinLength',20);


%     figure(figNum);
%     figNum = figNum + 1;
    imshow(img_zoomIn);
    hold on;
    
    lens = zeros(length(lines), 1);
    max_len = 0;
    
    angles = zeros(length(lines), length(lines));
    
    for line = 1:length(lines)
        xy = [lines(line).point1; lines(line).point2];
        plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');

        % Plot beginnings and ends of lines
        plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
        plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');

        % Determine the endpoints of the longest line segment
        len = norm(lines(line).point1 - lines(line).point2);
        lens(line) = len;
        if ( len > max_len)
            lineNum = line;
            max_len = len;
            xy_long = xy;
        end
        v1 = lines(line).point2 - lines(line).point1;
        for other = 1:length(lines)
            v2 = lines(other).point2 - lines(other).point1;
            angles(line, other) = round(rad2deg(acos(dot(v1, v2) / ...
                norm(v1) / norm(v2))));
        end
    end
    cylinderAngThrsh = 20;
    cylinderMeanAng = 0;
    meanDiffThrsh = 8;
    cubeAngThrsh = 80;
    cubeMeanAng = 45;

    maxAngle = max(max(angles))
    meanAngle = mean(mean(angles))
    if (maxAngle < cylinderAngThrsh && ...
            abs(meanAngle - cylinderMeanAng) < meanDiffThrsh)
        fprintf('cylinder\n');
        classification = 1;
    elseif (maxAngle > cubeAngThrsh && ...
            abs(meanAngle - cubeMeanAng) < meanDiffThrsh)
        fprintf('cube\n');
        classification = 3;
%     elseif 
%         
    end
    hold off
    
    
    
    
    if (stats.Area(k) > areaLowThrsh(statsNum) && ...
            major2minor < major2minorThrsh && ...
            stats.Area(k) < areaHighThrsh(statsNum))
        img_result = insertShape(img_result,'Rectangle', ...
            boundingBox, 'LineWidth',5);
%         img_result = insertText(img_result, stats.Centroid(k,:), ...
%             num2str(k));
    else
%         continue;
    end
    
    if (stats.Centroid(k,1) < startCoord(1))
        startCoord = stats.Centroid(k,:);
        startEndObjNum(1) = k;
    elseif (stats.Centroid(k,1) > endCoord(1))
        endCoord = stats.Centroid(k,:);
        startEndObjNum(2) = k;
    end
    
    % Cylinder Detection
    tolerance = 10;
    for n = 1:size(centers,1)
        dx = abs(stats.Centroid(k,1) - centers(n,1));
        dy = abs(stats.Centroid(k,2) - centers(n,2));
        centroid2circle = sqrt(dx^2 + dy^2);
        if (centroid2circle < radii(n) + tolerance || ...
                (centers(n,1) > boundingBox(1) && ...
                centers(n,1) < boundingBox(1) + boundingBox(3)) && ...
                centers(n,2) > boundingBox(2) && ...
                centers(n,2) < boundingBox(2) + boundingBox(4))
            numCylinder = numCylinder + 1;
            txt = strcat('Cylinder', sprintf('%02d',numCylinder));
            identified = 1;
            break;
        end
    end
    
    % Detection based on area
    triAreaThresh = triAreaBase;
    if (~identified)
        if (stats.Centroid(k,2) < 390)
            triAreaThresh = triAreaThresh - ...
                (390 - stats.Centroid(k,2)) * 19;
        else
            triAreaThresh = triAreaThresh + ...
                (stats.Centroid(k,2) - 390) * 12;
        end
        triAreaThresh = triAreaThresh - triAreaTolerance;
        if (stats.Area(k) > triAreaThresh(statsNum))
            numTriPrism = numTriPrism + 1;
            txt = strcat('Tri. Prism', sprintf('%02d',numTriPrism));
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
        txtLoc = [stats.Centroid(k,1) - 32, stats.Centroid(k,2) - 12];
        img_result = insertText(img_result, txtLoc, txt);
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
img_result = insertText(img_result, txtLoc1, txtCylinder);
img_result = insertText(img_result, txtLoc2, txtCube);
img_result = insertText(img_result, txtLoc3, txtTriPrism);

%% Non-Colliding Path
if (nonCollide)
    startObjSpace = cell2mat(stats.PixelList(startEndObjNum(1):startEndObjNum(1), 1:1));
    otherObjSpace = cell2mat(stats.PixelList(startEndObjNum(1)+1:startEndObjNum(2)-1, 1:1));
    endObjSpace = cell2mat(stats.PixelList(startEndObjNum(2):startEndObjNum(2), 1:1));
    fprintf('drawing X\n');
    pos = round(startCoord)
    img_result = insertShape(img_result, 'FilledCircle', ...
        [round(startCoord) 10], 'Color', 'black', 'Opacity', 1);
    fprintf('drawing O\n');
    round(endCoord)
    img_result = insertShape(img_result, 'FilledCircle', ...
        [round(endCoord) 10], 'Color', 'black', 'Opacity', 1);
    
    currentCoord = round(startCoord)
    path = currentCoord;
    reached = 0;
    step = 10;
    moved = 0;
    direction = 0;      % 0: moved normally; 1: up (neg); 2: down (pos)
    while (~reached)
        fprintf('----------------------------------------------\n');
        if (norm([endCoord(1) - currentCoord(1), ...
                endCoord(2) - currentCoord(2)]) < step + 10)
            reached = 1;
            break;
        end
        slope = (endCoord(2) - currentCoord(2)) / ...
            (endCoord(1) - currentCoord(1))
        %%%%%%start from here%%%%%%%
        nextX = ceil(currentCoord(1) + step);
        nextY = ceil(currentCoord(2) + step * slope);
        direction
        next
        if (direction == 0 && nextX <= endCoord(1))
            next = [ceil(currentCoord(1) + step) ...
                ceil(currentCoord(2) + step * slope)];
        elseif (direction == 1)
            nextStep = -step * 3;
            fprintf('a\n')
            if (ismember(currentCoord(1), path(:,1)) && ...
                    ismember(currentCoord(2) + nextStep, path(:,2)))
                fprintf('b\n')
                nextStep = step * 3;
            end
            next = [currentCoord(1) currentCoord(2) + nextStep];
        else
            nextStep = step;
            fprintf('c\n')
            if (ismember(currentCoord(1), path(:,1)) && ...
                    ismember(currentCoord(2) + nextStep, path(:,2)))
                fprintf('d\n')
                nextStep = -step;
            end
            next = [currentCoord(1) currentCoord(2) + nextStep];
        end
                
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         moved = 0;
%         nextPixel = impixel(img_propfilt2, next(1), next(2))
        [isStartObj, startIdx] = ismember(next, startObjSpace);
        [isOtherObj, otherIdx] = ismember(next, otherObjSpace);
        [isEndObj, endIdx] = ismember(next, endObjSpace);
        
%         if (isStartObj || isEndObj)
        idx1 = otherIdx(1) + size(otherObjSpace,1);
        idx2 = idx1 + size(otherObjSpace(otherObjSpace == next(1)),1);
        
        if (isOtherObj(1) && isOtherObj(2) && ...
                otherIdx(1) < size(otherObjSpace,1) / 2 && ...
                next(1) == otherObjSpace(otherIdx(1)) && ...
                ismember(next(2), otherObjSpace(idx1:idx2)))
            fprintf('next is otherObj\n');
            if (slope < 0 && moved) || (slope > 0 && ~moved) 
                fprintf('changing dir to 1: up\n');
                direction = 1;
            elseif (slope < 0 && ~moved) || (slope > 0 && moved)
                fprintf('changing dir to 2: down\n');
                direction = 2;
            end
            moved = 0;
        else
            fprintf('next is not otherObj\n');
            currentCoord = next;
            path = vertcat(path, currentCoord);
            direction = 0;
            moved = 1;
            fprintf('Drawing Path\n');
            currentCoord
%             img_result = insertText(img_result, currentCoord, '.', ...
%                 'FontSize', 30, 'TextColor', 'black', 'BoxOpacity', 0);
            img_result = insertShape(img_result, 'FilledCircle', ...
                [currentCoord 2], 'Color', 'black', 'Opacity', 1);
        end
        moved

%         dest = bitor(isStartObj, isEndObj)
%         if (impixel(img_propfilt2, next(1), next(2)) == dest)
%             currentCoord = next;
%             ifstate = 1
%         elseif (impixel(img_propfilt2, next(1), currentCoord(2)) == dest)
%             currentCoord = [next(1) currentCoord(2)];
%             ifstate = 2
%         elseif (impixel(img_propfilt2,currentCoord(1), next(2)) == dest)
%             currentCoord = [currentCoord(1), next(2)];
%             ifstate = 3
%         else
%             moved = 0;
%             ifstate = 4
% %             step = 1;
%         end
%         currentCoord
%         currentPixel = impixel(img_propfilt2, currentCoord(1), ...
%             currentCoord(2));
%         if (startObj && currentPixel(1) == 0)
%             fprintf('left startObj\n');
%             startObj = 0;
%         end
%         if (~startObj && currentPixel(1) == 1)
%             fprintf('entered endObj\n');
%             endObj = 1;
%         end
        
        
    end
    
end

%% Showing Resulting Image
figure(figNum);
figNum = figNum + 1;
imshow(img_result);

img = img_result;

% viscircles(centers, radii);
% viscircles(centers3, radii3);
% viscircles(domino_c, domino_r);
% end