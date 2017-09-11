close all;

level = '1. Basic Environment';
folder ='D:\Google Drive\UQ\Year 3\Semester 2\METR4202\ProblemSet2\';
if ~isdir(folder)
    folder = 'H:\METR4202\ProblemSet2\';
end

filePattern = fullfile(folder, level, '\*.bmp*');
file = dir(filePattern);
fileNames = {file.name};
figNum = 1;

for n = 1:numel(fileNames)
% 	imageFileNames{n} = fullfile(folder, fileNames{n});
    figure(figNum);
    figNum = figNum + 1;
    imshow(ps2_v2(fileNames{n}));
end