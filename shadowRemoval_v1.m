clear all;
close all;

%% Read Colourful Image
figure(1);
img_color = imread('greenlight3.bmp');
%imshow(img_color);

for n = 1:3
    rgbGray(:,:,n) = rgb2gray(img_color(:,:,n));
end

img_gray = rgb2gray(img_color);
imshow(img_gray);

%% Use Morphological Opening to Estimate the Background
background = imopen(img_gray,strel('octagon',30));
% Display the Background Approximation as a Surface
figure(2);
surf(double(background(1:8:end,1:8:end))),zlim([0 255]);
ax = gca;
ax.YDir = 'reverse';

%% Subtract the Background Image from the Original Image
figure(3);
img_bckgnd = img_gray - background;
imshow(img_bckgnd);

%% Increase the Image Contrast
figure(4);
img_contrast = imadjust(img_bckgnd);
imshow(img_contrast);

%% Threshold the Image
figure(5);
img_bw = imbinarize(img_contrast);
img_bw = bwareaopen(img_bw, 50);
imshow(img_bw)