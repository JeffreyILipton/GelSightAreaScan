function [ log, anns] = savePress(frames, framerate, folder, prefix, writeImages, ann)
%SAVEPRESS Save files for single press.

timestamp = datestr(datetime('now'), 'mm-dd-yy_HHMMss');
vidname = [prefix, ann.shape, ann.radius, '_', ann.hardness, '_', timestamp, '.avi'];
%vidname = [prefix, ann.x, ann.y, '_', timestamp, '.avi'];
vidful = [folder, '\', 'video', '\', vidname];

v = VideoWriter(vidful);
v.FrameRate = framerate;
open(v);
writeVideo(v, frames);
close(v);

ann.filename = ['video/', vidname];

log.files = {vidful};
log.num_images = 0;
anns.video = ann;
imgAnns = [];
if writeImages
    % Uncomment line 22 if want all images
    % for i = ann.start:ann.end
    for i = 1:ann.end % Comment out if you want all images
        % Following if loop is so that we only capture beginning and end
        % image for calibration purposes
        if (i ~= 1 && i~=ann.end-1)
            continue;
        end
        im = frames(:,:,:,i);
        imgname = [prefix, ann.shape, ann.radius, '_', ann.hardness, '_', num2str(i-ann.start), '_', timestamp, '.png'];
        %imgname = [prefix, ann.x, ann.y, '_', num2str(i-ann.start), '_', timestamp, '.png'];
        imgful = [folder, '\', 'image', '\', imgname];
        
        imwrite(im, imgful);
        
        imgann = ann;
        imgann.video_file = ann.filename;
        imgann.filename = ['image/', imgname];
        imgann.frame_idx = i;
%         imgann.height = size(im, 1);
%         imgann.width = size(im, 2);
        
        imgAnns = [imgAnns, imgann];
        log.files = [log.files, imgful];
        log.num_images = log.num_images + 1;
    end
    log.string = sprintf('Video file: %s has been created with %d images.\n', ...
        vidname, ann.end-ann.start+1);
else
    log.string = sprintf('Video file: %s has been created.\n', ...
        vidname);
end

anns.image = imgAnns;

log.string = sprintf('%sFrame Rate: %.2f fps\n', log.string, framerate);
log.string = sprintf('%sX-Coordinate (mm): %s\n', log.string, ann.hardness);
log.string = sprintf('%sY-Coordinate (mm): %s\n', log.string, ann.shape);
log.string = sprintf('%sOrientation (radians): %s\n', log.string, ann.radius);
%log.string = sprintf('%sX-Coordinate: %s\n', log.string, ann.x);
%log.string = sprintf('%sY-Coordinate: %s\n', log.string, ann.y);
log.string = sprintf('%sVideo Duration: %.2fs\n', log.string, size(frames, 4)/framerate);
log.string = sprintf('%sPress Duration: %.2fs\n', log.string, (ann.end-ann.start)/framerate);

