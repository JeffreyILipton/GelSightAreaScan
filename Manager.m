classdef Manager < handle
    %Manager Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(Access=public)
        optitrackSensor  % This object interfaces with the Sensor to collect data
        frameRate        % This is the rate at which the optitrack is querried
        framePeriod      % ?
        framePeriodSensor% ?
        
        gelSightSensor   % This is the object that interfaces with the GelSight

        dataLogger       % This object gets the data and writes it to a file
        
        body             % This object holds the location and quaternion of the optitrack and the offsets
        
        expType          % The type of the experiment
        expTimeSeconds   % The time of the experiment
        timestep
        
        debug

        abort
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
            obj.timestep = setup.timestep;
            obj.body = Body(setup.offset);
            obj.debug = debug;
            %OptitrackOnly, GelSightOnly,GelSightAndTracking, WithArm
            if(obj.expType == ExpTypes.OptitrackOnly)
                disp('[Manager] Optitrack Only Experiment');
                obj.optitrackSensor = OptitrackSensor(obj,obj.body);
                
                
                
                S = 1;
                N = floor(1/obj.framePeriod*(obj.expTimeSeconds)*2); % max data points to log
                obj.dataLogger = DataLogger(S, N, setup);

                
            elseif(obj.expType == ExpTypes.GelSightOnly)
                disp('[Manager] GelSight Only Experiment');
                obj.gelSightSensor = GelSight(setup.camNum);
                
                S = 1;
                N = floor(1/obj.framePeriod*(obj.expTimeSeconds)*1.2); % max data points to log
                obj.dataLogger = DataLogger(S, N, setup);

            elseif(obj.expType == ExpTypes.GelSightAndTracking)
                disp('[Manager] GelSight With position tracking Experiment ');
                obj.optitrackSensor = optitrackSensor(obj,obj.body);
                
                S = 1;
                N = floor(1/obj.framePeriod*(obj.expTimeSeconds)*2); % max data points to log
                obj.dataLogger = DataLogger(S, N, setup)

            elseif(obj.expType == ExpTypes.WithArm)
                disp('[Manager] Full Physical Experiment');

                %obj.environment = Environment();
                obj.optitrackSensor = optitrackSensor(obj,obj.body);
                obj.frameRate = obj.optitrackSensor.frameRate;
                obj.framePeriodSensor = 1/ obj.frameRate; %update framePeriod
                
                S = 1;
                N = floor(1/obj.framePeriod*(obj.expTimeSeconds)*1.2); % max data points to log
                obj.dataLogger = DataLogger(S, N, setup);
            end            
            
        end

        
        function start(obj)
            
            
            if(obj.expType ~= ExpTypes.GelSightOnly)
                % start optitrack
                l_result = obj.optitrackSensor.start();
                if(l_result ==1)
                    disp('[Manager] Error in Sensor occured');
                    obj.stop();
                    obj.delete();
                end
            end
            
            % start gelsight if its used
            if(obj.expType ~= ExpTypes.OptitrackOnly)
                obj.gelSightSensor.start();
                pause(0.1);
                obj.gelSightSensor.calibrate();
                obj.dataLogger.setCalibration(obj.gelSightSensor.calibrationImage);
                disp('Calibrated')
            end
            
            
            tRuntime = tic;
            % loop keeping track of the experiment length
            while(toc(tRuntime) < obj.expTimeSeconds && ~obj.abort)
                oneMeasurement = tic;

                Pos = NaN(1,3);
                Quat = NaN(1,4);
                
                if(obj.expType ~= ExpTypes.GelSightOnly)
                    % read from opti track
                    obj.optitrackSensor.getNewData(); % writes into armpcc
                    obj.sensorMeasurementsDone();
                    [Pos,Quat] = obj.body.getPosition([0,0,0]);
                end
                
                if(obj.expType ~= ExpTypes.OptitrackOnly)
                    obj.gelSightSensor.getNewData(Pos,Quat);
                    if obj.gelSightSensor.stage == 1
                        disp('PostProcessing');
                        [im,pos,quat] = obj.gelSightSensor.postProcess();
                        obj.dataLogger.addFrame(im,pos,quat);

                    elseif obj.gelSightSensor.stage == 2
                        if obj.debug
                            %name = datestr(now);%datestr(datetime('now'), 'mm-dd-yy_HHMMss');
                            filename = sprintf('data\\Sight_%s.mat', datestr(now));
                            filename = strrep(filename,':','_');
                            %folder = 'data'
                            %obj.gelSightSensor.savePress(im,folder,name);
                            Sight = obj.gelSightSensor;
                            save(filename,'Sight');
                            disp('[Manager] Saved Sensor');
                        end
                        obj.gelSightSensor.clear()
                        disp('[Manager] clear gelSight');
                    end
                end
                
                timeTaken = toc(oneMeasurement);
                if timeTaken < obj.timestep
                    pause(obj.timestep - timeTaken);
                end

                
            end

            obj.stop();
            obj.delete();
        end
        
        function stop(obj)
            disp('[Manager] Stopping Manager');
            % stop optitrack if using it
            if(obj.expType ~= ExpTypes.GelSightOnly)
                obj.optitrackSensor.stop();
            end
            
            % stop GelSight if using it
            if(obj.expType ~= ExpTypes.OptitrackOnly)
                obj.gelSightSensor.stop();
            end
            
        end
        
        function sensorMeasurementsDone(obj)
            obj.logData();            
        end
        
        function logData(obj)
            % All of the following gets logged
            [p,v] = obj.body.getPosition([0,0,0]);
            obj.dataLogger.add(p, v);
        end
        % Destructor
        function delete(obj)
            if isobject(obj.dataLogger)
                disp('[Manager] Wrap up Logging');
                % Save the shape history
                obj.dataLogger.postProcess();
                filename = sprintf('data\\DataLogger_%s.mat', datestr(now));
                filename = strrep(filename,':','_');
                History = obj.dataLogger;        %#ok
                save(filename, 'History');
                disp('[Manager] Log Saved');
                
                %delete shape history
                disp('[Manager] Deleting ShapeHistory');
                delete(obj.dataLogger);
            end
            
            if isobject(obj.optitrackSensor)
                obj.optitrackSensor.delete();
                disp('[Manager] Deleted Sensor');
            end
            
            if isobject(obj.gelSightSensor)
                if obj.debug
                    filename = sprintf('data\\Sight_%s.mat', datestr(now));
                    filename = strrep(filename,':','_');
                    %folder = 'data'
                    %obj.gelSightSensor.savePress(im,folder,name);
                    Sight = obj.gelSightSensor;
                    save(filename,'Sight');
                    disp('[Manager] Saved Sensor');
                end
                obj.gelSightSensor.delete();
                disp('[Manager] Deleted GelSight Sensor');
            end

        end
    end
end

