classdef SensorPCC < handle
    %SensorPCC Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        expType
        frameRate
        natNetClient
        frameListener
        simulationTimer
        listenerAttached
        frameOfData
        positionTime
        eulAngles
        thetaDotLPF
    end
    properties(Access=private)
        armPCC
        environment
        manager          % handle to parent class
        natNetClientInit
        totalNumRigidBodies
        filter_cut_off_frequency
        filter_sample_period
        hasEnvironment
        
    end
    
    methods(Access = public)
        % Constructor
        function obj = SensorPCC(managerHandle,armPCCHandle,environmentHandle)
            %Assign Handles
            obj.manager = managerHandle;
            obj.expType = obj.manager.expType;
            obj.armPCC = armPCCHandle;
            if nargin < 3
                obj.totalNumRigidBodies = obj.armPCC.numOfRigidBodies; 
                obj.hasEnvironment = false;
            else
                obj.environment = environmentHandle;
                % arm plus one for additional marker of the environment:
                obj.totalNumRigidBodies = obj.armPCC.numOfRigidBodies+obj.environment.numOfRigidBodies;
                obj.hasEnvironment = true;
            end
            
            %thetadot estimator
            obj.filter_cut_off_frequency = obj.manager.setup.filter_cut_off_frequency; %62.83
            disp('[SensorPCC] Frameperiod used for Low Pass Filter:')
            disp(num2str(obj.manager.framePeriod));
            obj.filter_sample_period  = obj.manager.framePeriod; % 0.0104 approx 1/100, from measurements
            obj.thetaDotLPF = LowPassFilter(obj.filter_cut_off_frequency,...
                obj.filter_sample_period, obj.armPCC.dims.S);
            
            obj.natNetClientInit = false;
            if(obj.expType ~= ExpTypes.Simulation)
            % Add NatNet .NET assembly so that Matlab can access its methods
            obj.createNatNetClient();
            % Connect to an OptiTrack server (Motive)
            obj.connectToOptiTrack();
            % print out a list of the active tracking Models in Motive
            %obj.getTrackedDataDescriptions()
            % get frame rate of tracking system
            obj.getFrameRate();
            end
        end
        function result = start(obj)
            obj.armPCC.absoluteTime = 0;
            result = 0;
            if(obj.expType == ExpTypes.Simulation)
                %result = obj.attachSimulationCallback();
            else
                % setup callback triggered whenever a new frame is received               
                %result = obj.attachFrameCallback();
            end
        end
        function getNewData(obj,simSensData)
            if(obj.expType ~= ExpTypes.Simulation)
                % Poll for latest frame instead of using event callback
                obj.frameOfData = obj.natNetClient.GetLastFrameOfData();
                frameTime = obj.frameOfData.fLatency;
                frameID = obj.frameOfData.iFrame;
                %if(frameTime ~= lastFrameTime)
                %    fprintf('FrameTime: %0.3f\tFrameID: %5d\n',frameTime, frameID);
                %    lastFrameTime = frameTime;
                %    lastFrameID = frameID;
                %else
                %    display('Duplicate frame');
                %end
            else
                obj.frameOfData = simSensData;              
            end
            
            %obj.frameOfData = event.data;
            %SensorPCC.helperDisplayDots(); %for debugging to display dots
            % do some work on that pass...
            obj.extractPositionData();
            
        end
        
        
        function stop(obj)
            if(obj.expType == ExpTypes.Simulation)
                %obj.detachSimulationCallback();
            else
                % delete callback triggered whenever a new frame is received                
                %obj.detachFrameCallback()
            end 
        end
        % Destructor
        function delete(obj)
            if(obj.expType == ExpTypes.Simulation)
                % stop callback
                %obj.detachSimulationCallback();
            else
                % cleanup
                %detach Frame Callback
                %obj.detachFrameCallback();
                
                %Uninitialize natNetClient
                obj.disconnectFromOptiTrack();
                
                %delete NatNetClient
                obj.destroyNatNetClient();
            end
        end
    end
    methods(Access = private)
        function createNatNetClient(obj)           
            % Add NatNet .NET assembly so that Matlab can access its methods, delegates, etc.
            % Note : The NatNetML.DLL assembly depends on NatNet.dll, so make sure they
            % are both in the same folder and/or path if you move them.
            disp('[SensorPCC] Creating NatNetClient.')
            % TODO : update the path to your NatNetML.DLL file here
            %dllPath = fullfile('c:','NatNetSDK2.5','lib','x64','NatNetML.dll');
            curDir = pwd;
            asmName = 'NatNetML';
            dllPath = fullfile(curDir,'..','3rd','NatNet_SDK_2.6','lib','x64',[asmName,'.dll']);
            % Check that the file exists
            assert(exist(dllPath, 'file') == 2, 'File does not exist');
            
            assemblyAlreadyExists = SensorPCC.IsAssemblyAdded(asmName);
            if(~assemblyAlreadyExists)
                NET.addAssembly(dllPath);
            end
            
            % Create an instance of a NatNet client
            obj.natNetClient = NatNetML.NatNetClientML(0); % Input = iConnectionType: 0 = Multicast, 1 = Unicast
            
            version = obj.natNetClient.NatNetVersion();
            fprintf( '[SensorPCC] Client Version : %d.%d.%d.%d\n', version(1), version(2), version(3), version(4) );
        end
        function destroyNatNetClient(obj)
            if(~isempty(obj.natNetClient))
                obj.natNetClient.delete();
                obj.natNetClient = [];
                disp('[SensorPCC] NatNetClient deleted!')
            else
                error('[SensorPCC] NatNetClient does not exist!');
            end       
        end
        
        function connectToOptiTrack(obj)
            % Connect to an OptiTrack server (e.g. Motive)
            disp('[SensorPCC] Connecting to OptiTrack Server (Motive).')
            if(~isempty(obj.natNetClient)&& obj.natNetClientInit == false)
                hst = java.net.InetAddress.getLocalHost;
                HostIP = char(hst.getHostAddress);
                %HostIP = char('128.30.27.47');
                flg = obj.natNetClient.Initialize(HostIP, HostIP); % Flg = returnCode: 0 = Success
                if (flg == 0)
                    obj.natNetClientInit = true;
                    disp('[SensorPCC] Connection to Server established')
                else
                    error('[SensorPCC] Connection to Server failed!')
                end
            else
                error('[SensorPCC] NatNetClient not created yet!')  
            end
        end
        
        function disconnectFromOptiTrack(obj)
            % Connect to an OptiTrack server (e.g. Motive)
            disp('[SensorPCC] Disconnecting from OptiTrack Server (Motive).')
            if(~isempty(obj.natNetClient) && obj.natNetClientInit == true)
                %Uninitialize natNetClient
                flg = obj.natNetClient.Uninitialize(); % Flg = returnCode: 0 = Success
                if (flg == 0)
                    obj.natNetClientInit = false;
                    disp('[SensorPCC] Succesfully disconnected from NatNetClient.')
                else
                    error('[SensorPCC] Disconnecting from NatNetClient failed!')
                end
            else
                error('[SensorPCC] Already disconnected from NatNetClient.');
            end
        end
        
        % Print out a description of actively tracked models from Motive
        function getTrackedDataDescriptions(obj) 
            %read data descriptions from nat net client
            dataDescriptions = obj.natNetClient.GetDataDescriptions();
            % print out
            fprintf('[SensorPCC] Tracking Models : %d\n\n', dataDescriptions.Count);
            for idx = 1 : dataDescriptions.Count
                descriptor = dataDescriptions.Item(idx-1);
                if(descriptor.type == 0)
                    fprintf('\tMarkerSet \t: ');
                elseif(descriptor.type == 1)
                    fprintf('\tRigid Body \t: ');
                elseif(descriptor.type == 2)
                    fprintf('\tSkeleton \t: ');
                else
                    fprintf('\tUnknown data type : ');
                end
                fprintf('%s\n', char(descriptor.Name));
            end
            
            for idx = 1 : dataDescriptions.Count
                descriptor = dataDescriptions.Item(idx-1);
                if(descriptor.type == 0)
                    fprintf('\n\tMarkerset : %s\t(%d markers)\n', char(descriptor.Name), descriptor.nMarkers);
                    markerNames = descriptor.MarkerNames;
                    for markerIndex = 1 : descriptor.nMarkers
                        name = markerNames(markerIndex);
                        fprintf('\t\tMarker : %-20s\t(ID=%d)\n', char(name), markerIndex);
                    end
                elseif(descriptor.type == 1)
                    fprintf('\n\tRigid Body : %s\t\t(ID=%d, ParentID=%d)\n', char(descriptor.Name),descriptor.ID,descriptor.parentID);
                elseif(descriptor.type == 2)
                    fprintf('\n\tSkeleton : %s\t(%d bones)\n', char(descriptor.Name), descriptor.nRigidBodies);
                    %fprintf('\t\tID : %d\n', descriptor.ID);
                    rigidBodies = descriptor.RigidBodies;
                    for boneIndex = 1 : descriptor.nRigidBodies
                        rigidBody = rigidBodies(boneIndex);
                        fprintf('\t\tBone : %-20s\t(ID=%d, ParentID=%d)\n', char(rigidBody.Name), rigidBody.ID, rigidBody.parentID);
                    end
                end
            end
            
        end
        
        function getFrameRate(obj)
            % send command/request to Motive to receive frame rate
            [byteArray, retCode] = obj.natNetClient.SendMessageAndWait(...
                'FrameRate');
            if(retCode == 0)
                byteArray = uint8(byteArray);
                obj.frameRate = typecast(byteArray,'single');
                fprintf('[SensorPCC] FrameRate: %i\n',obj.frameRate);
            else
                error('[SensorPCC] FrameRate not detected, Start Motive!\n');
            end
            
        end

        
        function result = attachFrameCallback(obj)
            % get the mocap data
            % approach 3 : get data by event handler (no polling)
            % Add NatNet FrameReady event handler
            result = 0;
            if(obj.natNetClientInit)
                if(isempty(obj.frameListener))
                    obj.frameListener = addlistener(obj.natNetClient,...
                        'OnFrameReady2',@(src,event)frameReadyCallback(obj,src,event));
                    disp('[SensorPCC] FrameReady Listener added.');
                else
                    disp('[SensorPCC] FrameReady Listener was already added before.');
                end
            else
                error('[SensorPCC] NatNet Client is not initialized.');
                result = 1;
            end
            
        end
        
        function detachFrameCallback(obj)
            if(obj.natNetClientInit)
                if(~isempty(obj.frameListener))
                    delete(obj.frameListener);
                    obj.frameListener = [];
                    disp('[SensorPCC] FrameReady Listener deleted.');
                else
                    disp('[SensorPCC] FrameReady Listener already deleted.');
                end
            else
                error('[SensorPCC] NatNet Client is not initialized, can not deattach.');
            end
        end
        
        function result = attachSimulationCallback(obj)
            % get fake mocap data
            % TODO run one time only
            %obj.manager.sensorMeasurementsDone();

            %ALternative: use timer:
%             if(isempty(obj.simulationTimer))
%                 obj.simulationTimer = timer('StartDelay', 1, 'Period', 1, 'TasksToExecute', Inf, ...
%                     'ExecutionMode', 'fixedRate');
%                 
%                 obj.simulationTimer.TimerFcn = {@(src,event)simulationCallback(obj,src,event)};
%                 start(obj.simulationTimer);
%                 
%                 disp('[SensorPCC] SimulationTimer added.');
%             else
%                 disp('[SensorPCC] SimulationTimer was already added before.');
%             end
            result = 0;
        end
        
        
        function detachSimulationCallback(obj)
            if(~isempty(obj.simulationTimer))
                stop(obj.simulationTimer);
                delete(obj.simulationTimer);
                obj.simulationTimer = [];
                disp('[SensorPCC] SimulationTimer deleted.');
            else
                disp('[SensorPCC] SimulationTimer  already deleted.');
            end
        end
        
        % Test : Process data in a NatNet FrameReady Event listener callback
        function frameReadyCallback(obj,src,event)
            
            obj.frameOfData = event.data;
            %SensorPCC.helperDisplayDots(); %for debugging to display dots
            % do some work on that pass...
            obj.extractPositionData();
        end
        
        function extractPositionData(obj)
            
            if(obj.expType ~= ExpTypes.Simulation)
                persistent p_lastFrameTime;
                persistent p_lastFrameID;
                persistent p_arrayIndex;
                persistent p_bufferModulo;

                % first time - generate an array and a plot
                if isempty(p_lastFrameTime)
                    % initialize statics
                    p_bufferModulo = 256;
                    p_arrayIndex = 1;
                    p_lastFrameTime = double(obj.frameOfData.fLatency);
                    p_lastFrameID = obj.frameOfData.iFrame;
                end

                % calculate the frame increment based on mocap frame's timestamp
                % in general this should be monotonically increasing according
                % To the mocap framerate, however frames are not guaranteed delivery
                % so to be accurate we test and report frame drop or duplication
                l_newFrame = true;
                l_droppedFrames = false;
                l_frameTime = double(obj.frameOfData.fLatency);

                l_frameID = obj.frameOfData.iFrame;
                if(l_frameID ==p_lastFrameID)
                    %debug
                    %fprintf('same id\n');
                end

                l_calcFrameInc = round( (l_frameTime - p_lastFrameTime) * obj.frameRate );
                %fprintf('%.12f l_frameTime\n',l_frameTime);
                %fprintf('%.12f p_lastFrameTime\n',p_lastFrameTime);

                % clamp it to a circular buffer of 255 frames
                p_arrayIndex = mod(p_arrayIndex + l_calcFrameInc, p_bufferModulo);
                if(p_arrayIndex==0)
                    p_arrayIndex = 1;
                end
                if(l_calcFrameInc > 1)
                    % debug
                    %fprintf('\nDropped Frame(s) : %d\n\tLastTime : %.3f\n\tThisTime : %.3f\n', l_calcFrameInc-1, p_lastFrameTime, l_frameTime);
                    l_droppedFrames = true;
                elseif(l_calcFrameInc == 0)
                    % debug
                    %fprintf('Duplicate Frame\n')
                    l_newFrame = false;
                end
            else
                l_newFrame = true;
                l_frameTime = NaN;
                l_frameID = NaN;
%                 obj.totalNumRigidBodies = obj.frameOfData.nRigidBodies;
            end
            
            % debug
            % fprintf('l_frameTime: %0.3f\tFrameID: %d\n',l_frameTime, l_frameID);
            
            try
                if(l_newFrame)
                    if(obj.expType == ExpTypes.Simulation || obj.frameOfData.nRigidBodies == obj.totalNumRigidBodies)
                        % RigidBodyData with properties:
                        % ID,x,y,z,qx,qy,qz,qw,nMarkers,Markers,MeanError,Tracked
                        %obj.positionTime = double(l_frameTime * obj.frameRate);
                        %obj.armPCC.framePeriod = l_calcFrameInc;
                        %obj.armPCC.absoluteTime = obj.armPCC.absoluteTime+l_calcFrameInc;
                        %disp(num2str(obj.armPCC.framePeriod));
                        
                        if(obj.expType ~= ExpTypes.Simulation)
                            i = 1;
                            % Extract info for arm and gripper
                            for s = 1:obj.totalNumRigidBodies
                                %X =  X in OptiTrack
                                posXVal =double(obj.frameOfData.RigidBodies(s).x);
                                %Y =  -Z in OptiTrack
                                posYVal = - double(obj.frameOfData.RigidBodies(s).z);
                                %Z =  Y in OptiTrack
                                posZVal = double(obj.frameOfData.RigidBodies(s).y);
                                rigidBodyData = obj.frameOfData.RigidBodies(s);

                                %rotate frame so z is up and x is along the
                                %arm
                                w = double(rigidBodyData.qw);
                                x = double(rigidBodyData.qx);
                                z = double(rigidBodyData.qy);
                                y = double(-rigidBodyData.qz);
                                %calculate yaw angle:
                                t3 = +2.0 * (w * z + x * y);
                                t4 = +1.0 - 2.0 * (y*y + z * z);
                                angleZ = atan2(t3, t4);
                                % equivalent to
                                %[~,~,angleZ] = SensorPCC.quaternion_to_euler_angle(q0,q1,q2,q3);

                                % CHANGE BELOW!
                                if (s >= 1 && s <= obj.armPCC.dims.S + 1)
                                    obj.armPCC.segPos2D(1,s) = posXVal;
                                    obj.armPCC.segPos2D(2,s) = posYVal;
                                    obj.armPCC.segPos2D(3,s) = posZVal;
                                    obj.armPCC.thetaMeasAbs(s) = angleZ;

                                elseif(s == obj.armPCC.dims.S + 2)
                                    % store end effector position to be stored in the environment 
                                    obj.environment.positionEndEffectorOT(1:2) = obj.armPCC.segPos2D(1:2,obj.armPCC.dims.S + 1);
                                    % store environment location
                                    obj.environment.posSineWaveStartOT(1) = posXVal;                                 
                                    obj.environment.posSineWaveStartOT(2) = posYVal;
                                    obj.environment.angleEnvironment = angleZ;
                                else
                                    disp('too many rigid bodies defined!!!!!');
                                end                           
                            end
                        else
                            % SIMULATION
                            for s = 1:(obj.armPCC.dims.S + 1)
                                obj.armPCC.segPos2D(1,s)   = obj.frameOfData(3*(s-1) + 1);
                                obj.armPCC.segPos2D(2,s)   = obj.frameOfData(3*(s-1) + 2);
                                obj.armPCC.segPos2D(3,s)   = 0;
                                obj.armPCC.thetaMeasAbs(s) = obj.frameOfData(3*(s-1) + 3);
                            end
                            % store end effector position to be stored in the environment 
                            obj.environment.positionEndEffectorOT(1:2) = [0 0];%obj.armPCC.segPos2D(1:2,obj.armPCC.dims.S + 1);
                            % store environment location
                            obj.environment.posSineWaveStartOT(1) = 0;                                 
                            obj.environment.posSineWaveStartOT(2) = 0;
                            obj.environment.angleEnvironment = 0;
                        end
                        %POST PRECESSING ARM - change
                        % substracting intial frame from 
                        obj.armPCC.segPos2D = obj.armPCC.segPos2D - repmat(obj.armPCC.segPos2D(:,1),1,obj.armPCC.dims.S + 1);  
                        %calculate relative end effector position by taking
                        %last column of the segpos2d
                        obj.armPCC.xMeas = obj.armPCC.segPos2D(:,obj.armPCC.dims.S + 1);
                        
                        %calculate relative angles
                        obj.armPCC.thetaMeas = diff(obj.armPCC.thetaMeasAbs);
                        %display last segments orientation

                        %disp(['RelAng:',num2str(obj.armPCC.thetaMeas/pi*180)]);
                        % Low pass filtered velocity
                        obj.armPCC.thetaDotMeas = obj.thetaDotLPF.calculate(obj.armPCC.thetaMeas);
                        %set absolute time and frame period
                        if obj.expType ~= ExpTypes.Simulation
                            obj.armPCC.setTimeAndFramePeriod();
                        else
                            obj.armPCC.absoluteTime = obj.frameOfData(end-1);
                        end
                        % Calibrate Arm and Gripper Values if not already
                        % done - Ceck for consistency in frames from
                        % optitrack and the arm being straight at the start
                        obj.armPCC.calibrateArm();
                        
                        % Tell Manager that measurements are done
                        obj.armPCC.newSensorValues = true;
                        %obj.manager.sensorMeasurementsDone();
                        
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%
                        % Debugging
                        %obj.detachFrameCallback();
                        %%%%%%%%%%%%%%%%%%%%%
                        
                        if(obj.hasEnvironment && obj.expType ~= ExpTypes.Simulation)
                            %% POST PROCESSING ENVIRONMENT to find contact directions
                            obj.environment.calculate();
                        elseif (obj.hasEnvironment && obj.expType == ExpTypes.Simulation)
                            obj.environment.setInContact(obj.frameOfData(end));%inContact = obj.frameOfData(end);
                            obj.environment.setContactDirectionOTNormed([0;1]);
                        end
                        
                    else
                        error('[SensorPCC] We have %i Rigid Bodies, but we need %i!\n',obj.frameOfData.nRigidBodies,obj.totalNumRigidBodies);
                    end
                end
            catch exc
                getReport(exc, 'extended')
                obj.detachFrameCallback();
                obj.manager.abort = true;
            end
            
            p_lastFrameTime = l_frameTime;
            p_lastFrameID = l_frameID;
            
        end
        function simulationCallback(obj,src,event)
            disp('Next Simulation Step!');
            %obj.positionTime = obj.positionTime+1;
            %obj.armPCC.framePeriod = double(1/(obj.frameRate));
            %obj.armPCC.absoluteTime = ?;
            
            for i = 1:obj.totalNumRigidBodies
            %TAKE X position
            obj.armPCC.segPos2D(1,i) = 0; %TODO fix this value to something reasonable
            %TAKE Y position
            obj.armPCC.segPos2D(2,i) = (i-1)*2.47*0.0254; % TODO fix this value
            end
            
            % Update Arm and Gripper Values
            %obj.armPCC.calculateSegmentValues(); % TODO: look if this ic calculated properly
            
            % Tell Manager that measurements are done
            %obj.manager.sensorMeasurementsDone();
        end
        
    end
    
    methods(Static)
        function angles = extractAnglesFromBody( rigidBody )
            q = quaternion( rigidBody.qx, rigidBody.qy, ...
                rigidBody.qz, rigidBody.qw );
            qRot = quaternion( 0, 0, 0, 1);     % rotate pitch 180 to avoid 180/-180 flip for nicer graphing
            q = mtimes(q, qRot);
            angles = EulerAngles(q,'zyx');
            angleX = -angles(1) * 180.0 / pi;   % must invert due to 180 flip above
            angleY = angles(2) * 180.0 / pi;
            angleZ = -angles(3) * 180.0 / pi;   % must invert due to 180 flip above
            angles = [angleX,angleY,angleZ];
        end
        function [X,Y,Z] = quaternion_to_euler_angle(w, x, y, z)
            ysqr = y * y;
            
            t0 = 2.0 * (w * x + y * z);
            t1 = 1.0 - 2.0 * (x * x + ysqr);
            X = atan2(t0, t1);
            
            t2 = +2.0 * (w * y - z * x);
            if t2 > +1.0
                t2 = +1.0;
            end
            if t2 < -1.0
                t2 = -1.0;
            end
            Y = asin(t2);
            
            t3 = +2.0 * (w * z + x * y);
            t4 = +1.0 - 2.0 * (ysqr + z * z);
            Z = atan2(t3, t4);
            
        end
        function helperDisplayDots( )
            persistent countDots;
            if isempty(countDots)
                countDots = 1;
            end
            % Code to display if framedata has been received
            fprintf('.');
            if countDots > 45
                fprintf('\n'); % That \n explicitly adds the linefeed
                countDots = 1;
            else
                countDots = countDots+1;
            end
        end
        function flag = IsAssemblyAdded( MyName )
            
            domain = System.AppDomain.CurrentDomain;
            assemblies = domain.GetAssemblies;
            flag = false;
            
            for i= 1:assemblies.Length
                
                asm = assemblies.Get(i-1);
                %disp(char(asm.FullName));
                asmName = char(asm.FullName);
                if strcmpi(asmName(1:length(MyName)), MyName)
                    flag = true;
                end
                
            end
        end
    end
    
end



