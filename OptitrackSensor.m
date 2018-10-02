classdef OptitrackSensor < handle
    %OptitrackSensor Summary of this class goes here
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
        body
    end
    properties(Access=private)
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
        function obj = OptitrackSensor(managerHandle,BodyHandle)
            %Assign Handles
            obj.manager = managerHandle;
            obj.expType = obj.manager.expType;
            obj.body = BodyHandle;
            obj.totalNumRigidBodies = 1; 
            
            %thetadot estimator
            %obj.filter_cut_off_frequency = obj.manager.setup.filter_cut_off_frequency; %62.83
            disp('[OptitrackSensor] Frameperiod used for Low Pass Filter:')
            disp(num2str(obj.manager.framePeriod));
            obj.filter_sample_period  = obj.manager.framePeriod; % 0.0104 approx 1/100, from measurements
            %obj.thetaDotLPF = LowPassFilter(obj.filter_cut_off_frequency,...
            %    obj.filter_sample_period, 1);
            
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
            %OptitrackSensor.helperDisplayDots(); %for debugging to display dots
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
            disp('[OptitrackSensor] Creating NatNetClient.')
            % TODO : update the path to your NatNetML.DLL file here
            %dllPath = fullfile('c:','NatNetSDK2.5','lib','x64','NatNetML.dll');
            curDir = pwd;
            asmName = 'NatNetML';
            dllPath = fullfile(curDir,'3rd','NatNet_SDK_2.6','lib','x64',[asmName,'.dll']);
            disp(curDir)
            % Check that the file exists
            assert(exist(dllPath, 'file') == 2, 'File does not exist');
            
            assemblyAlreadyExists = OptitrackSensor.IsAssemblyAdded(asmName);
            if(~assemblyAlreadyExists)
                NET.addAssembly(dllPath);
            end
            
            % Create an instance of a NatNet client
            obj.natNetClient = NatNetML.NatNetClientML(0); % Input = iConnectionType: 0 = Multicast, 1 = Unicast
            
            version = obj.natNetClient.NatNetVersion();
            fprintf( '[OptitrackSensor] Client Version : %d.%d.%d.%d\n', version(1), version(2), version(3), version(4) );
        end
        function destroyNatNetClient(obj)
            if(~isempty(obj.natNetClient))
                obj.natNetClient.delete();
                obj.natNetClient = [];
                disp('[OptitrackSensor] NatNetClient deleted!')
            else
                error('[OptitrackSensor] NatNetClient does not exist!');
            end       
        end
        
        function connectToOptiTrack(obj)
            % Connect to an OptiTrack server (e.g. Motive)
            disp('[OptitrackSensor] Connecting to OptiTrack Server (Motive).')
            if(~isempty(obj.natNetClient)&& obj.natNetClientInit == false)
                hst = java.net.InetAddress.getLocalHost;
                HostIP = char(hst.getHostAddress);
                %HostIP = char('128.30.27.47');
                flg = obj.natNetClient.Initialize(HostIP, HostIP); % Flg = returnCode: 0 = Success
                if (flg == 0)
                    obj.natNetClientInit = true;
                    disp('[OptitrackSensor] Connection to Server established')
                else
                    error('[OptitrackSensor] Connection to Server failed!')
                end
            else
                error('[OptitrackSensor] NatNetClient not created yet!')  
            end
        end
        
        function disconnectFromOptiTrack(obj)
            % Connect to an OptiTrack server (e.g. Motive)
            disp('[OptitrackSensor] Disconnecting from OptiTrack Server (Motive).')
            if(~isempty(obj.natNetClient) && obj.natNetClientInit == true)
                %Uninitialize natNetClient
                flg = obj.natNetClient.Uninitialize(); % Flg = returnCode: 0 = Success
                if (flg == 0)
                    obj.natNetClientInit = false;
                    disp('[OptitrackSensor] Succesfully disconnected from NatNetClient.')
                else
                    error('[OptitrackSensor] Disconnecting from NatNetClient failed!')
                end
            else
                error('[OptitrackSensor] Already disconnected from NatNetClient.');
            end
        end
        
        % Print out a description of actively tracked models from Motive
        function getTrackedDataDescriptions(obj) 
            %read data descriptions from nat net client
            dataDescriptions = obj.natNetClient.GetDataDescriptions();
            % print out
            fprintf('[OptitrackSensor] Tracking Models : %d\n\n', dataDescriptions.Count);
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
                fprintf('[OptitrackSensor] FrameRate: %i\n',obj.frameRate);
            else
                error('[OptitrackSensor] FrameRate not detected, Start Motive!\n');
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
                    disp('[OptitrackSensor] FrameReady Listener added.');
                else
                    disp('[OptitrackSensor] FrameReady Listener was already added before.');
                end
            else
                error('[OptitrackSensor] NatNet Client is not initialized.');
                result = 1;
            end
            
        end
        
        function detachFrameCallback(obj)
            if(obj.natNetClientInit)
                if(~isempty(obj.frameListener))
                    delete(obj.frameListener);
                    obj.frameListener = [];
                    disp('[OptitrackSensor] FrameReady Listener deleted.');
                else
                    disp('[OptitrackSensor] FrameReady Listener already deleted.');
                end
            else
                error('[OptitrackSensor] NatNet Client is not initialized, can not deattach.');
            end
        end
        
        
        % Test : Process data in a NatNet FrameReady Event listener callback
        function frameReadyCallback(obj,src,event)
            
            obj.frameOfData = event.data;
            %OptitrackSensor.helperDisplayDots(); %for debugging to display dots
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
                    if(obj.frameOfData.nRigidBodies == obj.totalNumRigidBodies)
                        % RigidBodyData with properties:
                        % ID,x,y,z,qx,qy,qz,qw,nMarkers,Markers,MeanError,Tracked
                        %obj.positionTime = double(l_frameTime * obj.frameRate);
                        %obj.armPCC.framePeriod = l_calcFrameInc;
                        %obj.armPCC.absoluteTime = obj.armPCC.absoluteTime+l_calcFrameInc;
                        %disp(num2str(obj.armPCC.framePeriod));

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
                            %t3 = +2.0 * (w * z + x * y);
                            %t4 = +1.0 - 2.0 * (y*y + z * z);
                            %angleZ = atan2(t3, t4);
                            % equivalent to
                            %[~,~,angleZ] = OptitrackSensor.quaternion_to_euler_angle(q0,q1,q2,q3);

                            % CHANGE BELOW!
                            if (s == 1)
                                obj.body.v = [posXVal,posYVal,posZVal];
                                obj.body.q = [w,x,y,z];
                            else
                                disp('too many rigid bodies defined!!!!!');
                            end                           
                        end
                       
                        
                        % Tell Manager that measurements are done
                        %obj.armPCC.newSensorValues = true;
                        %obj.manager.sensorMeasurementsDone();
                        
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%
                        % Debugging
                        %obj.detachFrameCallback();
                        %%%%%%%%%%%%%%%%%%%%%
                        

                        
                    else
                        error('[OptitrackSensor] We have %i Rigid Bodies, but we need %i!\n',obj.frameOfData.nRigidBodies,obj.totalNumRigidBodies);
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



