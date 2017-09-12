imagefiles = dir('1. Basic Environment/*.bmp');
nfiles = length(imagefiles);

images = cell(1, nfiles);
for i = 1:nfiles
    currentfilesname = fullfile('1. Basic Environment', imagefiles(i).name);
    currentimage = imread(currentfilesname);
    images{i} = im2double(rgb2gray(currentimage));
end

figure('Position', [100, 100, 1000, 700]);

set(gca, 'units', 'pixels');
sz = size(images{1});

cim = image( ...
    [1 sz(2)], ...
    [1 sz(1)], ...
    zeros(sz(1), sz(2), 1), ...
    'CDataMapping', 'scaled'...
    );

colormap gray;
axis image;

im = imgaussfilt(images{1}, 4);

set(cim, 'cdata', images{1});

drawnow;

template_rect = round(getrect);

template = im(...
    template_rect(2):(template_rect(2) + template_rect(4)), ...
    template_rect(1):(template_rect(1) + template_rect(3)) ...
    );

[result, it, res] = track_template(im, template, [template_rect(1); template_rect(2)]);
fprintf('Initialised track location to X: %g, Y: %g, it %i, r: %g\n', result(1), ...
    result(2), it, res);

result_handle = 0;
for i=  2:nfiles
    im = imgaussfilt(images{i}, 4);
    
    if result_handle ~= 0 
        delete(result_handle);
    end
    
    set(cim, 'cdata', images{i});
    
    [new_result, it, res] = track_template(im, template, result);
    
    if isnan(res)
        fprintf('Lost tracking ... initialising with previous result\n');
    else
        fprintf('Tracked template to X: %g, Y: %g, it: %i, r: %g\n', result(1), ...
            result(2), it , res);
    end
    
    if ~any(isnan(new_result))
        result = new_result;
    end
    
    hold on;
    result_handle = scatter(result(1), result(2));
    hold off;
    
    drawnow;
end