%% Extracting DataLogger image data and saving it as .png files

f = 'C:\Users\GelSight\Documents\GitHub\GelSightAreaScan\data\';
files = dir(strcat(f, '*.mat'));

for file = 1:length(files)
    % Loads data as variable called 'History'
    clear History im imgfile pic;
    load(strcat(f, files(file).name));
    
    fprintf("Whooo I loaded something\n");
    
    % Iterates through the images loaded from 'History'
    for pic = 1:length(History.framePos)
        %imshow(History.frames(:,:,:,pic)); % Shows images
        im = History.frames(:,:,:,pic);
        imgfile = sprintf('%s_%d.png', strcat(f, files(file).name(1:end-4)), pic);
        
        imwrite(im, imgfile);
    end

end