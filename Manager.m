classdef Manager < handle
    %Manager Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(Access=public)
        armPCC;
        baseBoard;
        environment;
        sensorPCC;
        forceSensor;
        trajGen;
        controllerPCC;
        simulationTime;
        shapeHistoryPCC;
        expType;
        
        armPlotHandleTarget;
        armPlotHandleMeas;
        objectPlotHandle;
        frameRate
        framePeriod
        framePeriodSensor
        
        expTimeSeconds
        abort
        numContactForces
        setup
    end
    
    methods(Access = public)
        % Constructor
        function obj = Manager(setup)
            obj.setup = setup;
            if(isa(setup.expType,'ExpTypes'))
                obj.expType = setup.expType;
            else
                error('wrong type');
            end
            obj.abort = false;
            obj.expTimeSeconds = setup.expTimeSeconds;
            obj.framePeriod = setup.framePeriod;
            
            
            if(obj.expType == ExpTypes.Sensing)
                disp('[Manager] Sensing Experiment');
                %no actuation
                actuated = false;
                obj.armPCC = ArmPCC( actuated , setup.armDof, ...
                    setup.gripperExists,obj.numContactForces,setup);
                obj.sensorPCC = SensorPCC(obj,obj.armPCC);
                %design trajectory evolution over time
                %[xd_vect, dxd_vect, ddxd_vect,time_vect] = obj.trajectory(setup);
                
            elseif(obj.expType == ExpTypes.Characterization)
                disp('[Manager] Characterization Experiment');
                % actuation,sensing,characterizer (incl.logging)
                actuated = true; % TODO FIX THIS BACK TO TRUE
                obj.armPCC = ArmPCC( actuated , setup.armDof, ...
                    setup.gripperExists,obj.numContactForces,setup);
                obj.sensorPCC = SensorPCC(obj,obj.armPCC);
                
                obj.frameRate = obj.sensorPCC.frameRate;
                obj.framePeriodSensor = 1/ obj.frameRate; %update framePeriod
                obj.controllerPCC = CharacterizerPCC(obj.armPCC,setup.characterizerForce,setup.segmentNumber);
                
                % Logging
                
                S = obj.armPCC.dims.S; %dimension (segments)
                N = floor(1/obj.framePeriod*(obj.expTimeSeconds)*2); % max data points to log
                obj.shapeHistoryPCC = ShapeHistoryPCC(S, N, setup);
                %Logging
                %obj.simulationTime = minutes*60;
                %numOfTimeSteps = obj.simulationTime * obj.sensorPCC.frameRate;
                %obj.shapeHistoryPCC = ShapeHistoryPCC(obj.armPCC.dims.S, numOfTimeSteps);
                
            elseif(obj.expType == ExpTypes.Simulation)
                disp('[Manager] NOT DEBUGGED!!! Simulation Experiment ');
                actuated = false;
                obj.armPCC = ArmPCC( actuated , setup.armDof, ...
                    setup.gripperExists,obj.numContactForces,setup);
                
                obj.environment = Environment();
                obj.sensorPCC = SensorPCC(obj,obj.armPCC,obj.environment);
                %obj.trajGen = TrajGen();
                %frameRate = 100;
                %obj.framePeriodSensor = 1/frameRate;
                L_vect_in = 0;
                limb_in = 0;
                d_vect_in = 0;
                xd_vect_in = 0;
                dxd_vect_in = 0;
                ddxd_vect_in = 0;
                time_vect_in=0;
                
                actuated = true; %TODO: make actuated
                
                %obj.environment = Environment();
                
                % Create the robot (run startup before)
                obj.armPCC.createRobot();
                
%                 obj.controllerPCC = ControllerPCC(setup,...
%                     obj.armPCC,...
%                     L_vect_in,limb_in,d_vect_in,...
%                     xd_vect_in,dxd_vect_in,ddxd_vect_in,time_vect_in);
                
                %design trajectory evolution over time
                [xd_vect, dxd_vect, ddxd_vect,time_vect] = obj.trajectory(setup);
                
                %transpose input vectors
                obj.controllerPCC = ControllerPCC(setup, obj.armPCC,...
                    xd_vect,dxd_vect,ddxd_vect,time_vect,obj.environment);
            elseif(obj.expType == ExpTypes.PhysicalExperiment)
                disp('[Manager] Physical Experiment');
                actuated = true; %TODO: make actuated
                obj.armPCC = ArmPCC( actuated , setup.armDof, ...
                    setup.gripperExists,obj.numContactForces,setup);
                %obj.environment = Environment();
                obj.sensorPCC = SensorPCC(obj,obj.armPCC);
                obj.frameRate = obj.sensorPCC.frameRate;
                obj.framePeriodSensor = 1/ obj.frameRate; %update framePeriod
                activeForceSensor = true;
                
                % Create the robot (run startup before)
                obj.armPCC.createRobot();
                
                
                %design trajectory evolution over time
                [xd_vect, dxd_vect, ddxd_vect,time_vect] = obj.trajectory(setup);
                
                %transpose input vectors
                obj.controllerPCC = ControllerPCC(setup, obj.armPCC,...
                    xd_vect,dxd_vect,ddxd_vect,time_vect);
                

                S = obj.armPCC.dims.S; %dimension (segments)
                N = floor(1/obj.framePeriod*(obj.expTimeSeconds)*1.2); % max data points to log
                obj.shapeHistoryPCC = ShapeHistoryPCC(S, N, setup);
            elseif(obj.expType == ExpTypes.PhysicalExperimentEnvironment)
                disp('[Manager] Physical Experiment with Environment');
                obj.armPCC = ArmPCC( setup.actuatorActive , setup.armDof, ...
                    setup.gripperExists,obj.numContactForces,setup);
                obj.environment = Environment(setup.environmentType,setup.plotting);
                obj.sensorPCC = SensorPCC(obj,obj.armPCC,obj.environment);
                obj.frameRate = obj.sensorPCC.frameRate;
                obj.framePeriodSensor = 1/ obj.frameRate; %update framePeriod
                activeForceSensor = false;
                
                % Create the robot (run startup before)
                obj.armPCC.createRobot();
                
                %design trajectory evolution over time
                [xd_vect, dxd_vect, ddxd_vect,time_vect] = obj.trajectory(setup);
                
                %transpose input vectors
                obj.controllerPCC = ControllerPCC(setup, obj.armPCC,...
                    xd_vect,dxd_vect,ddxd_vect,time_vect,obj.environment);
                
                S = obj.armPCC.dims.S; %dimension (segments)
                N = floor(1/obj.framePeriod*(obj.expTimeSeconds)*1.2); % max data points to log
                obj.shapeHistoryPCC = ShapeHistoryPCC(S, N, setup);
            end
            obj.forceSensor = ForceSensor(obj.framePeriod,obj.numContactForces,activeForceSensor);
            
            
        end
%         function [x_vect, dx_vect, ddx_vect, time_vect] = trajectory(obj,setup)
%             timeTotal = setup.expTimeSeconds; %s
%             
%             steps = floor(timeTotal/obj.framePeriod);
%             time_vect = linspace(0,timeTotal,steps)';
%             
%             if(setup.controllerType == ControllerTypes.CurvatureLevelControl)
%                 
%                 fact = 2*pi*1/timeTotal*setup.numPeriods;
%                 arg = time_vect*fact;
%                 if(strcmp(setup.profile,'-cos'))
%                 
%                 %             x_vect = setup.amplitude*ones(steps,obj.armPCC.dims.S);
%                 %             dx_vect = zeros(steps,obj.armPCC.dims.S);
%                 %             ddx_vect = zeros(steps,obj.armPCC.dims.S);
%                 x_vect = repmat(-setup.amplitude*cos(arg)+setup.offset,1,obj.armPCC.dims.S);
%                 dx_vect = repmat(setup.amplitude*sin(arg)*fact,1,obj.armPCC.dims.S);
%                 ddx_vect = repmat(setup.amplitude*cos(arg)*fact^2,1,obj.armPCC.dims.S);
%                 
%                 elseif(strcmp(setup.profile,'s_shape'))
%                   x_vect = kron(-setup.amplitude*cos(arg)+setup.offset, [1 1 1 -1 -1 -1]);%-[-1 -1 -1 -1 2 2]);%[2 -1 -1 -1 -1 2]);
%                  dx_vect = kron( setup.amplitude*sin(arg)*fact  , [1 1 1 -1 -1 -1]);
%                 ddx_vect = kron( setup.amplitude*cos(arg)*fact^2, [1 1 1 -1 -1 -1]);
%                 elseif(strcmp(setup.profile,'4_tip'))
%                 x_vect = kron(-setup.amplitude*cos(arg)+setup.offset, [0 0 1 1 1 1]);%-[-1 -1 -1 -1 2 2]);%[2 -1 -1 -1 -1 2]);
%                 dx_vect = kron( setup.amplitude*sin(arg)*fact  , [0 0 1 1 1 1]);
%                 ddx_vect = kron( setup.amplitude*cos(arg)*fact^2, [0 0 1 1 1 1]);
%                 end
%                 %poses = amplitude*sin(arg)';
%                 %plot(t,poses)
%                 %positions = repmat(poses,1,obj.numSegments);
%                 %positions = zeros(steps,obj.armPCC.dims.S);
%                 %positions(:,6) = poses;
%             elseif(setup.controllerType == ControllerTypes.CartesianImpedanceControl)
%                 x_vect = repmat(setup.phaseOneCartPoseDes,steps,1);
%                 dx_vect = zeros(size(x_vect));
%                 ddx_vect = dx_vect;
%             elseif(setup.controllerType == ControllerTypes.CartesianImpedanceAdvancedControl)
%                 x_vect = repmat(setup.phaseOneCartPoseDes,steps,1);
%                 dx_vect = zeros(size(x_vect));
%                 ddx_vect = dx_vect;
%             end
%             
%         end
        function start(obj)
            
            % start optitrack
            l_result = obj.sensorPCC.start();
            if(l_result ==1)
                disp('[Manager] Error in Sensor occured');
                obj.stop();
                obj.delete();
            end
           
            %try
            if(obj.expType ~= ExpTypes.Sensing)
                l_result = obj.controllerPCC.start();
            end
            %catch
            if(l_result ==1)
                disp('[Manager] Error in Controller occured');
                obj.stop();
                obj.delete();
            end
            tRuntime = tic;
            %obj.armPCC.contactForce = [0,0,0];
            % loop keeping track of the experiment length
            if obj.expType ~= ExpTypes.Simulation
                while(toc(tRuntime) < obj.expTimeSeconds && ~obj.abort)
                    oneMeasurement = tic;

                    % read from opti track
                    obj.sensorPCC.getNewData(); % writes into armpcc
                    obj.sensorMeasurementsDone();
                    %end
                    timeTaken = toc(oneMeasurement);

                    pause(0.01 - timeTaken);
                end
                
                obj.stop();
                obj.delete();
            else
                % Simulation
                %sim('dummy');
            end
        end
        function stop(obj)
            disp('[Manager] Stopping Manager');
            if(obj.expType ~= ExpTypes.Sensing)
                obj.controllerPCC.stop();
            end
            obj.sensorPCC.stop();
            obj.forceSensor.stop();
        end
        function sensorMeasurementsDone(obj)
                        
            if(obj.expType == ExpTypes.PhysicalExperiment)
                if(obj.armPCC.calibrated == true)
                    l_result = obj.controllerPCC.control();
                    obj.logData();
                else
                    l_result = 0;
                end
         
                if(l_result == 1)
                    obj.stop();
                    obj.delete();
                end
            elseif(obj.expType == ExpTypes.PhysicalExperimentEnvironment)
                if(obj.armPCC.calibrated == true)
                    l_result = obj.controllerPCC.control();
                    obj.logData();
                else
                    l_result = 0;
                end
                
                if(l_result == 1)
                    obj.stop();
                    obj.delete();
                end
            elseif(obj.expType == ExpTypes.Simulation)
                l_result = obj.controllerPCC.control();
                if(l_result == 1)
                    obj.stop();
                    obj.delete();
                end
            elseif(obj.expType == ExpTypes.Sensing)
                %TODO: nothing here yet
            elseif(obj.expType == ExpTypes.Characterization)
                if(obj.armPCC.calibrated == true)
                    %l_result = obj.controllerPCC.step();
                    l_result = obj.controllerPCC.ramp();
                    %obj.controllerPCC.newSensorData = true;
                    obj.logData();
                else
                    l_result = 0;
                end
      
                if(l_result == 1)
                    obj.stop();
                    obj.delete();
                end
            end
        end
        
        function logData(obj)
            % All of the following gets logged
            obj.shapeHistoryPCC.add(...
                obj.armPCC.absoluteTime, obj.armPCC.forceTarget, obj.controllerPCC.torque',...
                obj.armPCC.thetaMeas, obj.armPCC.thetaDotMeas,...
                obj.controllerPCC.th_ref, obj.controllerPCC.dth_ref,obj.controllerPCC.ddth_ref,...
                obj.armPCC.segPos2D, NaN(1,2),obj.armPCC.framePeriodMeas,obj.armPCC.contactForce);
        end
        % Destructor
        function delete(obj)
            if isobject(obj.shapeHistoryPCC)
                disp('[Manager] Wrap up Logging');
                % Save the shape history
                obj.shapeHistoryPCC.postProcess();
                filename = sprintf('data\\%s.mat', datestr(now));
                filename = strrep(filename,':','_');
                History = obj.shapeHistoryPCC;        %#ok
                save(filename, 'History');
                disp('[Manager] Log Saved');
                
                %delete shape history
                disp('[Manager] Deleting ShapeHistory');
                delete(obj.shapeHistoryPCC);
            end
            
            %disp('[Manager] Deleting Manager');
            if isobject(obj.armPCC)
                obj.armPCC.delete();
                disp('[Manager] Deleted Arm');
            end
            if isobject(obj.environment)
                obj.environment.delete();
                disp('[Manager] Deleted Environment');
            end
            if isobject(obj.sensorPCC)
                obj.sensorPCC.delete();
                disp('[Manager] Deleted Sensor');
            end
            if isobject(obj.forceSensor)
                obj.forceSensor.delete();
                disp('[Manager] Deleted ForceSensor');
            end
            
            if isobject(obj.controllerPCC)
                obj.controllerPCC.delete();
                disp('[Manager] Deleted Controller');
            end
            
            %             if isobject(obj.shapeHistoryPCC)
            %                 obj.shapeHistoryPCC.delete();
            %             end
        end
    end
end

