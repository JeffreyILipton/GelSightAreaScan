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
        ther_mul1
        ther_mul2
        maxd
        buffersize
        
        newDataAvalible
        
    end
    
    methods
        function obj = GelSight(camNum)
            %GELSIGHT Construct an instance of this class
            %   Detailed explanation goes here
            obj.vid= videoinput('winvideo', camNum, 'RGB24_640x480');
            
            obj.ther_mul1 = 3;
            obj.ther_mul2 = 0.5;
            obj.buffersize = 15;
            obj.maxd = [];
            obj.init = [];
            obj.frames = [];
            obj.times = [];
            obj.deltas=[];

        end
        
        function start(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            atobj = getselectedsource(vid);
            set(vid, 'FramesPerTrigger', 1);
            set(vid, 'TriggerRepeat', Inf);
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
                obj.newDataAvalible = false;
            else
                
            end
        end        
        
        
        
    end
end

