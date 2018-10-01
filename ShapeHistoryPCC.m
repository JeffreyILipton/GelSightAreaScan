classdef ShapeHistoryPCC < handle
    %SHAPEHISTORY Class containing the history of the arm's shape
    %   This is a handle class, i.e. objects are always passed by reference
    %   rather than value. As a result the values can be efficiently
    %   updated on the go without copying the whole history.
    
    properties
        S   % Number of segments in the arm
        N   % Size of samples buffer
        i   % Current index
        
        timestamps      % Timestamps of the state
        forceTarget
        torqueTarget
        thetaMeas
        thetaDotMeas
        xd
        dxd
        ddxd
        %kMeas
        %arcLenMeas
        segmentPositions
        environmentPosition       
        %controllerResults
        framePeriod
        framePeriodAvg
        experimentSetup
        contactForces
    end
    
    methods
        function obj = ShapeHistoryPCC (S, N, experimentSetup)
            % Create the initial object
            obj.S = S; %number of segments
            obj.N = N; %number of time steps
            obj.i = 0;
            
            obj.timestamps = NaN(N,1);            
            obj.forceTarget = NaN(N,S);
            obj.torqueTarget = NaN(N,S);
            obj.thetaMeas = NaN(N,S);
            obj.thetaDotMeas = NaN(N,S);
            if(experimentSetup.controllerType == ControllerTypes.CurvatureLevelControl)
            obj.xd = NaN(N,S);
            obj.dxd = NaN(N,S);
            obj.ddxd = NaN(N,S);
            elseif(experimentSetup.controllerType == ControllerTypes.CartesianImpedanceControl ||...
                    experimentSetup.controllerType == ControllerTypes.CartesianImpedanceAdvancedControl)
            obj.xd = NaN(N,length(experimentSetup.phaseOneCartPoseDes));
            obj.dxd = NaN(N,length(experimentSetup.phaseOneCartPoseDes));
            obj.ddxd = NaN(N,length(experimentSetup.phaseOneCartPoseDes));
            else
                error('this case does not exist');
            end
            %obj.kMeas = NaN(N,S);
            %obj.arcLenMeas = NaN(N,S);
            obj.segmentPositions = NaN(N,3,S + 1);
            obj.environmentPosition = NaN(N,2);
            obj.framePeriod = NaN(N,1);
            obj.contactForces = NaN(N,experimentSetup.numContactForces);
            %obj.controllerResults = struct();
            obj.experimentSetup = experimentSetup;
            
        end
        
        function add(obj, timestamp, forceTarget,torqueTarget,thetaMeas, thetaDotMeas,...
                xd,dxd,ddxd,...
                segmentPositions, environmentPosition,framePeriod,contactForces)
            % Update the count
            obj.i = mod(obj.i + 1, obj.N);
            if obj.i == 0
                obj.i = obj.N;
            end
            % Add a new measurement
            obj.timestamps(obj.i, 1) = timestamp;
            obj.forceTarget(obj.i, :) = forceTarget;
            obj.torqueTarget(obj.i, :) = torqueTarget;
            obj.thetaMeas(obj.i, :) = thetaMeas;
            obj.thetaDotMeas(obj.i, :) = thetaDotMeas;
            obj.xd(obj.i, :) = xd;
            obj.dxd(obj.i, :) = dxd;
            obj.ddxd(obj.i, :) = ddxd;
            obj.segmentPositions(obj.i,:,:) = segmentPositions;
            obj.environmentPosition(obj.i, :) = environmentPosition;
            obj.framePeriod(obj.i, :) = framePeriod;
            obj.contactForces(obj.i, :) = contactForces;
        end
        
        function postProcess(obj)
            obj.framePeriodAvg = mean(obj.framePeriod(1:obj.i));
        end
%         function addControllerResults(obj,tuning)
%             % Add controller results
%             obj.controllerResults.tuning = tuning;
%         end
                
    end
    
end
