classdef ForceControllerHost < handle
    %ForceControllerHost Communicating with Simulink Controller on second PC
    % The embedded curvature controller is actually implemented as a SIMULINK
    % simulation running on another Windows PC machine. The communication
    % with the controller is established via a serial link with the below
    % properties.
    
    properties
        udpPort
        numSegments
        actuated
        init
        %preload
        %noPreload
        %forceScaling
    end
    
    methods
        function obj = ForceControllerHost(numSegments,actuated)
            obj.numSegments = numSegments;
            obj.actuated = actuated;
            obj.init = false;
            if(obj.actuated)
                % Establish the serial port communication
                disp('[ForceControllerHost] Opening UDP Connection')
                %obj.defineLimits();
                
                obj.initUdp();
                disp('[ForceControllerHost] Opened Serial Ports')
            end
        end
        %         function defineLimits(obj)
        %             % for all of the following values, first row is left side of
        %             % the arm (when looking from the root along the arm), second
        %             % row is right side of the arm
        %             obj.preload = [2000,1900,2000,1900,1900,1900;
        %                                       2000,1900,2000,1900,1900,1900];%uint8([216, 43]);
        %             obj.forceScaling = [1.1,1,1.1,1,1,1;...
        %                                            1.1,1,1.2,1,1,1];
        %
        %             obj.noPreload = 1000*ones(2,obj.serialInfo.numSegments);%uint8([200, 31]);
        %             obj.maxPos = [1700,1700,1700,1700,1700,1700;
        %                                       1700,1700,1700,1700,1700,1700];%2500*ones(1,obj.serialInfo.numSegments);%uint8([200, 31]);
        %         end
        function initUdp(obj)
            disp('Opening UDP Ports')
            clientip = '128.30.25.105';
            %clientip = 'drl-2darm';
            %hostip = '128.30.25.104';
            %hostip = 'drl-cc'
            clientport = 8866;
            hostport = 8844;
            obj.udpPort = udp(clientip, 'RemotePort',clientport , 'LocalPort', hostport);
            
            notOpen = true;
            while(notOpen)
                try
                    fopen(obj.udpPort);
                    notOpen = false;
                catch
                    warning('[ForceControllerHost] UDP port already opened, need to close first!')
                    %instrfind
                    instrreset
                end
            end
            zeroPosition = zeros(1,obj.numSegments);
            obj.sendPositions( zeroPosition );
            obj.init = true;
            disp('[ForceControllerHost] Opened UDP Port')
            %zeroPosition = zeros(1,obj.serialInfo.numSegments);
            %obj.moveToPosition( zeroPosition, obj.serialInfo.preload );
            %disp('[ForceControllerHost]  Preloaded Actuators')
            
            
        end
        
        function testActuation(obj,amplitude,segmentNumber)
            try
                
                %initialize
                disp('Move in sinusoid');
                timeTotal = 3; %s
                period = 0.012;
                steps = floor(timeTotal/period);
                t = linspace(0,timeTotal,steps);
                arg = t*2*pi*1/timeTotal;
                poses = amplitude*sin(arg)';
                %plot(t,poses)
                %
               
                if(segmentNumber < 1 || segmentNumber > 7)
                    positions = repmat(poses,1,obj.numSegments);
                else
                    positions = zeros(steps,obj.numSegments);
                    positions(:,segmentNumber) = poses;
                end
                timeTaken = zeros(steps,1);
                tstart = tic;
                for k = 1:steps
                    just = tic;
                    obj.sendPositions( positions(k,:));
                    timeTaken(k) = toc(just);
                    pause(period-timeTaken(k))
                end
                toc(tstart)
                mean(timeTaken)
                disp('Done Moving')
            catch
                obj.deinitUdp();
                return
            end
        end
        
        function sendPositions( obj, target )
            %sendPositions Sends position to the embedded PID motor controller
            %   INPUT:
            %   obj         Serial Port Object - curvature controller object
            %   target      1xS vector - target curvatures
            
            %error checking
            l_targetSize = size(target);
            if( l_targetSize(2) ~= obj.numSegments && l_targetSize(1) == 1)
                error('[FCH] function input target has wrong size');
            end
            if(obj.actuated)
                % Send the command
                fwrite(obj.udpPort,int16(target),'int16');
            else
                %TODO: plot or log command for simulation
            end
            
        end
        
        function deinitUdp(obj)
            %deinitialize
            if (obj.actuated)
                if(obj.init)
                    zeroPosition = zeros(1,obj.numSegments);
                    obj.sendPositions( zeroPosition );
                    fclose(obj.udpPort);
                    delete(obj.udpPort)
                    obj.init = false;
                end
            end
        end
        
        function delete(obj)
            %DELETE Disconnects from the curvature controller
            %   The embedded curvature controller is actually implemented as a SIMULINK
            %   simulation running on another Windows PC machine. The communication
            %   with the controller is establisher via a serial link with the below
            %   properties.
            if (obj.actuated)
                % Close the serial port communication
                disp('[ForceControllerHost] Closing UDP Port')
                obj.deinitUdp();
                disp('[ForceControllerHost] Closed UDP Port')
            end
        end
    end
end
