classdef GelSight
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
        
    end
    
    methods
        function obj = GelSight(camNum)
            %GELSIGHT Construct an instance of this class
            %   Detailed explanation goes here
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
            obj.calibrationImage = [];
            avg_num = 2;
            for i = 1:avg_num
                if isempty(obj.calibrationImage)
                    obj.calibrationImage = getsnapshot(obj.vid) / avg_num;
                else
                    obj.calibrationImage = obj.calibrationImage + getsnapshot(obj.vid) / avg_num;
                end
            end
            start(obj.vid);
        end
        
        function getNewData(obj)
            if ~obj.vid.FramesAvailable
                obj.newDataAvailable = false;
            else
                [f, time] = getdata(obj.vid, get(obj.vid, 'FramesAvailable'));
                fend = f(:,:,:,end);
                pause(0.01);
                if ~obj.newDataAvailable
                    stop(obj.vid);
                end
            end
                del_pic = abs(fend(:) - obj.calibrationImage);
                del_pic(del_pic < 10) = 0;
                delta = repmat(sum(del_pic), size(time));
            if isempty(obj.frames)
                obj.deltas = delta;
            elseif ob.stage == 0
                obj.frames = cat(4, obj.frames(:,:,:,max(end-obj.buffersize, 1):end), f);
                obj.times = [obj.times(max(end-obj.buffersize, 1):end); time];
                obj.deltas = [obj.deltas(max(end-obj.buffersize, 1):end); delta];
            else
                obj.frames = cat(4, obj.frames, f);
                obj.times = [obj.times; time];
                obj.deltas = [obj.deltas; delta];
            test(obj.deltas(end));
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
        
        function stageinput(obj)
            if obj.stage == 0
                if obj.deltas(end) > max(obj.init * max(obj.thre_mul1, 2), 1e6);
                    obj.stage = 1;
                    obj.newDataAvailable = true;
                end
            end
            if obj.stage == 1
                if obj.deltas(end) < obj.maxd * obj.thre_mul2
                    obj.newDataAvailable = false;
                    obj.thre_mul1 = obj.maxd / obj.init / 8;
                    pause(0.01);
            
                    
        end
        
    end
end

