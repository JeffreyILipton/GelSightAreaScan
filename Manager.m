classdef Manager < handle
    %Manager Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(Access=public)
        optitrackSensor  % This object interfaces with the Sensor to collect data
        frameRate        % This is the rate at which the optitrack is querried

        
        gelSightSensor   % This is the object that interfaces with the GelSight

        dataLogger       % This object gets the data and writes it to a file
        
        body             % This object holds the location and quaternion of the optitrack and the offsets
        
        expType          % The type of the experiment
        expTimeSeconds   % The time of the experiment
        timestep
        
        hsa
        
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
            obj.timestep = setup.timestep;
            obj.body = Body(setup.offset);
            obj.debug = setup.debug;
            
            obj.dataLogger = DataLogger(setup);
            
            
            %OptitrackOnly, GelSightOnly,GelSightAndTracking, WithArm
            if(obj.expType == ExpTypes.OptitrackOnly)
                disp('[Manager] Optitrack Only Experiment');
                obj.optitrackSensor = OptitrackSensor(obj,obj.body);
                obj.frameRate = obj.optitrackSensor.frameRate;
                

                
            elseif(obj.expType == ExpTypes.GelSightOnly)
                disp('[Manager] GelSight Only Experiment');
                obj.gelSightSensor = GelSight(setup.camNum);

            elseif(obj.expType == ExpTypes.GelSightAndTracking)
                disp('[Manager] GelSight With position tracking Experiment ');
                obj.optitrackSensor = optitrackSensor(obj,obj.body);
                obj.frameRate = obj.optitrackSensor.frameRate;
                obj.gelSightSensor = GelSight(setup.camNum);

            elseif(obj.expType == ExpTypes.WithArm)
                disp('[Manager] Full Physical Experiment');

                %obj.environment = Environment();
                obj.optitrackSensor = optitrackSensor(obj,obj.body);
                obj.frameRate = obj.optitrackSensor.frameRate;
                obj.gelSightSensor = GelSight(setup.camNum);
                
                obj.hsa = HSA(setup.port,setup.mins,setup.maxs);

            end            
            
        end

        
        function start(obj)
            
            
            if(isobject(obj.optitrackSensor))
                % start optitrack
                l_result = obj.optitrackSensor.start();
                if(l_result ==1)
                    disp('[Manager] Error in Sensor occured');
                    obj.stop();
                    obj.delete();
                end
            end
            
            % start gelsight if its used
            if(isobject(obj.gelSightSensor))
                obj.gelSightSensor.start();
                pause(0.1);
                obj.gelSightSensor.calibrate();
                obj.dataLogger.setCalibration(obj.gelSightSensor.calibrationImage);
                disp('[Manager] GelSight Calibrated')
            end
            
            
            tRuntime = tic;
            % loop keeping track of the experiment length
            while(~obj.abort)
                oneMeasurement = tic;
                
                %If a time limit is set, check if we should continue
                if( obj.expTimeSeconds)
                    if (toc(tRuntime) > obj.expTimeSeconds)
                        break
                    end
                end
                
                

                Pos = NaN(1,3);
                Quat = NaN(1,4);
                
                %update Position and Quaternion from sensor
                if(isobject(obj.optitrackSensor))
                    % read from opti track
                    obj.optitrackSensor.getNewData(); % writes into armpcc
                    obj.sensorMeasurementsDone();
                    [Pos,Quat] = obj.body.getPosition([0,0,0]);
                end
                
                %run gelsight sensor
                if(isobject(obj.gelSightSensor))
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
            if(isobject(obj.optitrackSensor) )
                obj.optitrackSensor.stop();
            end
            
            % stop GelSight if using it
            if(isobject(obj.gelSightSensor) )
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
                disp('[Manager] Deleting History');
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

