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
        
        simObj
        hwObj
        pts
        
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

            elseif(obj.expType == ExpTypes.TestHSA)
                disp('[Manager] Test HSA+Gelsight Experiment');

                obj.gelSightSensor = GelSight(setup.camNum);
                
                obj.hsa = HSA(setup.HSA_port,setup.HSA_channels,setup.HSA_mins,setup.HSA_maxs);
            
            elseif(obj.expType == ExpTypes.TestARM)
                disp('[Manager] Test Arm Experiment');
                %obj.environment = Environment();
                
                obj.simObj = URsim;
                obj.simObj.Initialize;
                    simObj.FrameT = Tz(160);
    
                % Hide frames
                frames = '0123456E';
                for i = 1:numel(frames)
                    hideTriad(simObj.(sprintf('hFrame%s',frames(i))));
                end
                % Connect to hardware
                % -> The message to the user *assumes* that you have:
                %       (1) Properly installed URToolboxScript on the UR controller. See
                %       the link below for instructions:
                %       
                %       https://www.usna.edu/Users/weapsys/kutzer/_Code-Development/UR_Toolbox.php
                %
                %       (2) Configure the network card connecting the PC to the UR 
                %       controller to a static IP of 10.1.1.5
                %
                %       (3) Set the UR controller IP to 10.1.1.2
                if setup.useHardware
                    instruct = sprintf([...
                        '\tPython module imported.\n',...
                        '\tEnter server IP address: 192.168.1.105\n',... 
                        '\tEnter port: 30002\n',...
                        '\tEnter number of connections to be made: 1\n',...
                        '\tServer created.\n',...
                        '\tBegin onboard controller, then press ENTER.\n',...
                        '\tConnections established.\n',...
                        '\tWould you like to create a URX connection as well? y\n',...
                        '\tEnter URX address: 192.168.1.106\n',... %10.1.1.2
                        '\tURX connection established.\n']);
                    fprintf('PLEASE USE THE FOLLOWING RESPONSES:\n\n');
                    fprintf(2,'%s\n\n',instruct)

                    obj.hwObj = UR;
                end
                
                % Create path
                obj.pts = makeWayPoints(setup.origin,setup.xysize,setup.delta);
                % Transform coordinates into the workspace of the robot
                obj.pts = Tz(500)*Rx(pi/2)*Tz(500)*obj.pts;
                
            elseif(obj.expType == ExpTypes.WithArm)
                disp('[Manager] Full Physical Experiment');

                %obj.environment = Environment();
                obj.optitrackSensor = optitrackSensor(obj,obj.body);
                obj.frameRate = obj.optitrackSensor.frameRate;
                obj.gelSightSensor = GelSight(setup.camNum);
                
                obj.hsa = HSA(setup.HSA_port,setup.HSA_mins,setup.HSA_maxs);
                
                obj.pts = makeWayPoints(setup.origin,setup.xysize,setup.delta);
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
            
            if(isobject(obj.hsa))
                obj.hsa.start()
                pause(0.5);
                obj.hsa.setPos(0.0);
                pause(1.0);
            end

            if(isobject(obj.simObj))
                obj.simObj.Home;
                if(isObject(obj.hwObj))
                    q = obj.simObj.Joints;
                    msg(obj.hwObj,sprintf('(%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f)',q,zeros(6,1)));
                    UR_WaitForMove(obj.hwObj);
                end
                drawnow
            end
            
            ptNum=1;
            moveNext = true;
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
                
                if(isobject(obj.simObj))
                    if (ptNum>length(obj.pts))
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
                
                if(isobject(obj.simObj) && moveNext)
                    % Define pose from waypoint
                    H_cur = Tx(obj.pts(1,ptNum))*Ty(obj.pts(2,ptNum))*Tz(obj.pts(3,ptNum))*Rx(pi/2);
                    % Set simulation toolpose to waypoint pose
                    obj.simObj.ToolPose = H_cur;
                    % Move robot to match simulation
                    q = obj.simObj.Joints;
                    if (isobject(obj.hwObj))
                        % Get joint position from the simulation
                        q = obj.simObj.Joints;
                        % Send waypoint to the robot
                        msg(obj.hwObj,sprintf('(%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f)',q,zeros(6,1)));
                        % Wait for move only on the first waypoint

                        % Wait for the robot to finish executing the move
                        UR_WaitForMove(obj.hwObj);

                    end
                    ptNum= ptNum+1;
                    moveNext = false;
                    % Allow plot to update
                    drawnow;
                end
                
                %run gelsight sensor
                if(isobject(obj.gelSightSensor))
                    obj.gelSightSensor.getNewData(Pos,Quat);
                    if obj.gelSightSensor.stage == 0
                        % Looking for rise
                        if isobject(obj.hsa)
                            obj.hsa.setPos(1.0);
                        end
                    elseif obj.gelSightSensor.stage == 1
                        %It has risen
                        if ( (obj.gelSightSensor.deltas(end) > 12000000) || ( mean(diff(obj.gelSightSensor.deltas(end-10:end)))< 10000 ) )
                            % pull back
                            if isobject(obj.hsa)
                                obj.hsa.setPos(0.0);
                            end
                        end
                    elseif obj.gelSightSensor.stage == 2
                        if isobject(obj.hsa)
                            obj.hsa.setPos(0.0);
                        end
                        %Pressure falling
                        disp('PostProcessing');
                        [im,pos,quat] = obj.gelSightSensor.postProcess();
                        obj.dataLogger.addFrame(im,pos,quat);

                    elseif obj.gelSightSensor.stage == 3
                        % done processing
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
                        if isobject(obj.hsa)
                            obj.hsa.setPos(0.0);
                            pause(1.0);
                        end
                        moveNext= true;
                    end
                    
                    
                else
                    moveNext= true;
                end
                

                
                
                timeTaken = toc(oneMeasurement);
                if timeTaken < obj.timestep
                    pause(obj.timestep - timeTaken);
                end

                if(0==isobject(obj.gelSightSensor))
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
            
            % stop HSA
            if(isobject(obj.hsa))
                %obj.hsa.stop();
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
            
            if(isobject(obj.hsa))
                obj.hsa.setPos(0);
                pause(1);
                obj.hsa.stop();
                obj.hsa.delete();
                disp('[Manager] Deleted HSA');
            end
            
            if(isobject(obj.simObj))
                obj.simObj.delete();
                disp('[Manager] Deleted UR sim');
            end
            
            if(isobject(obj.hwObj))
                obj.hwObj.delete();
                disp('[Manager] Deleted UR hw');
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
