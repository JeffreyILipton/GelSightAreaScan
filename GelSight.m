classdef GelSight < handle
    %GELSIGHT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        vid
        
        init
        frames
        times
        deltas
        
        calibrationImage
        thre_mul1
        thre_mul2
        maxd
        buffersize
        
        newDataAvailable 
        stage
        start_index
        
    end
    
    methods
        function obj = GelSight(camNum)
            %GELSIGHT Construct an instance of this class
            %   Detailed explanation goes here
            imaqreset;
            obj.vid= videoinput('winvideo', camNum, 'RGB24_640x480');
            
            obj.thre_mul1 = 3;
            obj.thre_mul2 = 0.5;
            obj.buffersize = 15;
            obj.maxd = [];
            obj.init = [];
            obj.frames = [];
            obj.times = [];
            obj.deltas=[];
            obj.newDataAvailable = false;
            obj.stage = 0;
            obj.start_index = 0;
        end
        
        function start(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            atobj = getselectedsource(obj.vid);
            set(obj.vid, 'FramesPerTrigger', 1);
            set(obj.vid, 'TriggerRepeat', Inf);
            set(atobj,'Exposure',0);
        end
        
        function calibrate(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            im = [];
            avg_num = 2;
            for i = 1:avg_num
                if isempty(im)
                    im = getsnapshot(obj.vid) / avg_num;
                else
                    im = im + getsnapshot(obj.vid) / avg_num;
                end
            end
            obj.calibrationImage = im;
            start(obj.vid);
        end
        
        function getNewData(obj)
            if ~obj.vid.FramesAvailable
                obj.newDataAvailable = false;
            else
                [f, time] = getdata(obj.vid, get(obj.vid, 'FramesAvailable'));
                fend = f(:,:,:,end);
                pause(0.01);
                del_pic = abs(fend(:) - obj.calibrationImage(:));
                del_pic(del_pic < 10) = 0;
                delta = repmat(sum(del_pic), size(time));
                if isempty(obj.frames)
                    obj.frames = f;
                    obj.times = time;
                    obj.deltas = delta;
%                 elseif obj.stage == 0
%                     obj.frames = cat(4, obj.frames(:,:,:,max(end-obj.buffersize, 1):end), f);
%                     obj.times = [obj.times(max(end-obj.buffersize, 1):end); time];
%                     obj.deltas = [obj.deltas(max(end-obj.buffersize, 1):end); delta];
                else
                    obj.frames = cat(4, obj.frames, f);
                    obj.times = [obj.times; time];
                    obj.deltas = [obj.deltas; delta];
                end
                maxdelta(obj);
                stageinput(obj);
                obj.newDataAvailable = true;
                disp('deltas: ')
                delta
                disp('num deltas: ')
                length(obj.deltas)
                disp(['stage: ',num2str(obj.stage)])
            end
        end        
        
        function maxdelta(obj)
            if isempty(obj.init)
                obj.init = obj.deltas(end);
            end
            if isempty(obj.maxd)
                obj.maxd = obj.deltas(end);
            end
            if obj.deltas(end) > obj.maxd
                obj.maxd = obj.deltas(end);
            end
        end
        
        function stop(obj)
            stop(obj.vid);
        end
        
        function stageinput(obj)
            if obj.stage == 0
                % find start of rise
                if obj.deltas(end) > max(obj.init * max(obj.thre_mul1, 2), 1e6)
                    %obj.stage = 1;
                    obj.start_index = length(obj.times);
                    obj.newDataAvailable = true;
                end
                
                %detect fall
                if obj.deltas(end) < obj.maxd * obj.thre_mul2
                    obj.newDataAvailable = false;
                    if obj.deltas(end) < obj.init * obj.thre_mul1
                        obj.stage = 1;
                    end
                end
                
            end  
        end
        
        function postProcess(obj)
            obj.times = obj.times - obj.times(obj.start_index);
            name = datestr(datetime('now'), 'mm-dd-yy_HHMMss');
            folder = 'data'
            savePress(obj,folder,name);
            obj.stage=2;
        end
        
        
        function clear(obj)
            obj.maxd = 0;
            obj.calibrationImage = obj.frames(:,:,:,1);
            obj.frames = [];
            obj.times = [];
            obj.deltas = [];
            obj.newDataAvailable = false;
            flushdata(obj.vid);
            obj.stage=0;
        end
        
        function imgful = savePress(obj,folder,name)
            [~, xmax] = max(obj.deltas);
            
            im = obj.frames(:,:,:,xmax);
            
            imgname = [name, '.png'];
            fullfolder = [folder, '\', 'image'];
            imgful = [fullfolder, '\', imgname];
            
            write = true;
            if 7~= exist(fullfolder)
                write = mkdir(fullfolder);
            end
            if write
                imwrite(im, imgful);
            end
        end
        
        function vidname = saveVideo(obj,folder,name)
            framerate = mean(1./diff(obj.times));
            [~, xmax] = max(obj.deltas);
            
            fullfolder = [folder,'\','video'];
            vidname = [fullfolder,'\', name, '.avi'];
            
            write = true;
            if 7~= exist(fullfolder)
                write = mkdir(fullfolder);
            end
            if write
                v = VideoWriter(vidname);
                v.FrameRate = framerate;
                open(v);
                %Cutframes = obj.frames(:,:,:,obj.start_index:xmax);
                %writeVideo(v,Cutframes);
                writeVideo(v,obj.frames);
                close(v);
            end
            

        end
        
        
    end
end

