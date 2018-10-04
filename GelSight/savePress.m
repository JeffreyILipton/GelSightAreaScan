function savePress(frames, framerate, ann)
%SAVEPRESS Save files for single press.

timestamp = datestr(datetime('now'), 'mm-dd-yy_HHMMss');
vidname = [timestamp, '.avi'];

v = VideoWriter(vidname);
v.FrameRate = framerate;
open(v);
writeVideo(v, frames);
close(v);

ann.filename = ['video/', vidname];

anns.video = ann;
imgAnns = [];
if writeImages
    % Uncomment line 23 if want all images
    % for i = ann.start:ann.end
    for i = 1:ann.end % Comment out if you want all images
        % Following if loop is so that we only capture beginning and end
        % image for calibration purposes
        if (i ~= 1 && i~=ann.end-1)
            continue;
        end
        im = frames(:,:,:,i);
        imgname = [num2str(i-ann.start), '_', timestamp, '.png'];
        imgful = [folder, '\', 'image', '\', imgname];
        
        imwrite(im, imgful);
        
        imgann = ann;
        imgann.video_file = ann.filename;
        imgann.filename = ['image/', imgname];
        imgann.frame_idx = i;
%         imgann.height = size(im, 1);
%         imgann.width = size(im, 2);
        
        imgAnns = [imgAnns, imgann];
    end
end

anns.image = imgAnns;
