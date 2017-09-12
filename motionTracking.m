close all;

level = '4. Videos';

folder ='D:\Google Drive\UQ\Year 3\Semester 2\METR4202\ProblemSet2\';
if ~isdir(folder)
    folder = 'H:\METR4202\ProblemSet2\';
end
filePattern = fullfile(folder, level, '\*.avi*');
file = dir(filePattern);
fileNames = {file.name};

for n = 1:numel(fileNames)
	videoFileNames{n} = fullfile(folder, level, fileNames{n});
end

videoSource = VideoReader(videoFileNames{1});
videoFrame = readFrame(videoSource);

% imshow(objDetect(videoFrame))
while hasFrame(v)
    videoFrame = readFrame(videoSource);
end





% videoSource = vision.VideoFileReader(videoFileNames{1});

% videoFrame = step(videoSource);

% detector = vision.ForegroundDetector;
% 
% blob = vision.BlobAnalysis(...
%        'CentroidOutputPort', false, 'AreaOutputPort', false, ...
%        'BoundingBoxOutputPort', true, ...
%        'MinimumBlobAreaSource', 'Property', 'MinimumBlobArea', 500);
%    
% shapeInserter = vision.ShapeInserter('BorderColor','White');
% videoPlayer = vision.VideoPlayer();
% while ~isDone(videoSource)
%      frame  = step(videoSource);
%      fgMask = step(detector, frame);
%      bbox   = step(blob, fgMask);
%      out    = step(shapeInserter, frame, bbox);
%      step(videoPlayer, out);
% end

% release(videoPlayer);
% release(videoSource);
