classdef DataLogger < handle
    %SHAPEHISTORY Class containing the history of the arm's shape
    %   This is a handle class, i.e. objects are always passed by reference
    %   rather than value. As a result the values can be efficiently
    %   updated on the go without copying the whole history.
    
    properties
        S   % Number of segments in the arm
        N   % Size of samples buffer
        i   % Current index
        
        timestamps      % Timestamps of the state
        positions
        quaternions
        
        calibration
        frames
        framePos
        frameQuat
        
        framePeriod
        framePeriodAvg
        experimentSetup
    end
    
    methods
        function obj = DataLogger (S, N, experimentSetup)
            % Create the initial object
            obj.S = S; %number of segments
            obj.N = N; %number of time steps
            obj.i = 0;
            
            obj.calibration = [];
            obj.timestamps = NaN(N,1);            
            obj.positions = NaN(N,3);
            obj.quaternions = NaN(N,4);
            obj.framePeriod = NaN(N,1);
            obj.experimentSetup = experimentSetup;
            obj.frames = [];
            obj.framePos = [];
            obj.frameQuat = [];
        end
        
        function setCalibration(obj,calibration)
            obj.calibration = calibration;
        end
        
        function add(obj, position, quaternion)
            % Update the count
            obj.i = mod(obj.i + 1, obj.N);
            if obj.i == 0
                obj.i = obj.N;
            end
            % Add a new measurement
            %obj.timestamps(obj.i, 1) = timestamp;
            
            obj.positions(obj.i, :) = position;
            obj.quaternions(obj.i, :) = quaternion;
            %obj.framePeriod(obj.i, :) = framePeriod;

        end
        
        function addFrame(obj, frame, position, quaternion)
            if isempty(obj.frames)
                obj.frames = frame;
                obj.framePos = position;
                obj.frameQuat = quaternion;
            else
                obj.frames = cat(4, obj.frames, frame);
                obj.framePos = [obj.framePos; position];
                obj.frameQuat = [obj.frameQuat; quaternion];
            end

        end
        
        
        function postProcess(obj)
            obj.framePeriodAvg = mean(obj.framePeriod(1:obj.i));
        end

                
    end
    
end
