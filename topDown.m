function img_transform = topDown(img_gray, movingPoints, fixedPoints)
%     figure(1);
%     imshow(img_gray)
    
%     if (exist('cpselectPts'))
%         load('cpselectPts');
%     end

    
    tform = cp2tform(movingPoints, fixedPoints, 'projective');
    
    img_transform = imtransform(img_gray, tform, 'XYScale', 0.15);
    
%     img_crop = igft(round(size(igft, 1) * 0.7): end, :);
%     img_crop = img_crop(:, round(size(img_crop, 2) * 0.1) : ...
%         round(size(img_crop, 2) * 0.5));
    
%     figure(2);
%     imshow(igft);
    
end