classdef ForceControllerClient < handle
    %ForceControllerClient Communicating with Simulink Controller on second PC
    % The embedded curvature controller is actually implemented as a SIMULINK
    % simulation running on another Windows PC machine. The communication
    % with the controller is established via a serial link with the below
    % properties.
    
    properties
        serialInfo
        vectorLength
        
        actuated
    end

    methods
        function obj = ForceControllerClient(vecLength,actuated)
            obj.vectorLength = vecLength;
            
            obj.serialInfo = struct();
            obj.actuated = actuated;
            if(obj.actuated)
                % Establish the serial port communication
                disp('[ForceControllerClient] Opening Serial Ports')
                obj.defineSerialInfo();
                
                obj.initSerial();
                disp('[ForceControllerClient] Opened Serial Ports')
            end
        end
        function defineSerialInfo(obj)
            obj.serialInfo.portName = {...
                'COM19','COM22',...
                'COM23','COM25',...
                'COM17','COM16',...
                'COM27','COM29',...
                'COM31','COM34',...
                'COM35','COM38'}; %M6 left,right
            
            obj.serialInfo.numPorts = length(obj.serialInfo.portName);
            obj.serialInfo.numSegments = obj.serialInfo.numPorts/2;
            if( obj.serialInfo.numSegments ~= obj.vectorLength)
                error('[ForceControllerClient] vectorLength input  has wrong dimension');
            end
            % for all of the following values, first row is left side of
            % the arm (when looking from the root along the arm), second
            % row is right side of the arm
            % These values need recalibration for each arm, starting at
            % value 1900
            % these values need to be so tight that a 100 max force
            % amplitude is visible
            obj.serialInfo.preload = [2350,1950,2250,2250,1950,1950;
                                      1900,2000,2100,2150,2000,2100];%uint8([216, 43]);

            obj.serialInfo.forceScaling = [1.05, 1, 1.09 , 1.05 , 1 , 0.96;...
                                           1   , 1, 1    , 1    , 1 , 1.04];
            
            obj.serialInfo.noPreload = 1000*ones(2,obj.serialInfo.numSegments);%uint8([200, 31]);
            %obj.serialInfo.maxPos = [1700,1700,1700,1700,1700,1700;
            %                          1700,1700,1700,1700,1700,1700];%2500*ones(1,obj.serialInfo.numSegments);%uint8([200, 31]);
            obj.serialInfo.maxPos = 1050*ones(2,obj.serialInfo.numSegments);%uint8([200, 31]);

            obj.serialInfo.serialPorts = cell(obj.serialInfo.numPorts,1);
        end
        function initSerial(obj)
            disp('Opening Serial Ports')
            m =1;
            while m<=obj.serialInfo.numPorts
                %display(['Opening Serial Port on ',obj.serialInfo.portName{i}])
                obj.serialInfo.serialPorts{m} = serial(obj.serialInfo.portName{m});
                set(obj.serialInfo.serialPorts{m},'BaudRate',57600,'DataBits',8,'StopBits',1,'Parity','none');
                fclose(obj.serialInfo.serialPorts{m});
                try
                    fopen(obj.serialInfo.serialPorts{m});
                    m = m+1;
                catch ME
                    warning('Serial Ports already opened, need to close first!')
                    instrfind
                    instrreset
                    m = 1;
                end
                    %display(['Opened Serial Port on ',obj.serialInfo.portName{i}])
            end
            obj.serialInfo.init = 1;
            disp('[ForceControllerClient] Opened Serial Ports')
            zeroPosition = zeros(1,obj.serialInfo.numSegments);
            obj.moveToPosition( zeroPosition, obj.serialInfo.preload );
            disp('[ForceControllerClient]  Preloaded Actuators')
            
            
        end
        function moveToPosition( obj, positions, preload )       
            for j = 1:obj.serialInfo.numSegments
                [m1l, m1h, m2l, m2h] = ForceControllerClient.posToMotorCmd(...
                    positions(j),preload(:,j),obj.serialInfo.maxPos(:,j),...
                    obj.serialInfo.forceScaling(:,j),j);
                fwrite(obj.serialInfo.serialPorts{(j-1)*2+1}, [m1l, m1h], 'uint8');
                fwrite(obj.serialInfo.serialPorts{(j-1)*2+2}, [m2l, m2h], 'uint8');
            end            
        end        
        
        
        function testActuation(obj)
            try
                %initialize
                disp('Move in sinusoid');
                numSegments = obj.serialInfo.numSegments;
                amplitude = 600;
                timeTotal = 2; %s
                steps = floor(timeTotal*120);
                t = linspace(0,timeTotal,steps);
                arg = t*2*pi*1/timeTotal;
                poses = amplitude*sin(arg)';
                %plot(t,poses)
                %positions = repmat(poses,1,numSegments);
                positions = zeros(steps,numSegments);
                positions(:,6) = poses;
                
                for k = 1:steps
                    %obj.moveToPosition(positions(k,:),obj.serialInfo.preload);

                    obj.sendPositions(positions(k,:))
                    pause(timeTotal/steps)
                end
                
                disp('Done Moving')
            catch   
                obj.deinitSerial();
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
            if( l_targetSize(2) ~= obj.vectorLength && l_targetSize(1) == 1)
                error('ForceControllerClient function input target has wrong size');
            end
            if(obj.actuated)
                % Send the command
                obj.moveToPosition(target,obj.serialInfo.preload);
            else
                %TODO: plot or log command for simulation
            end
            
        end
        
        
        
        function deinitSerial(obj)
            %UNTITLED3 Summary of this function goes here
            %   Detailed explanation goes here
            
            %deinitialize
            if (obj.actuated)
                if(obj.serialInfo.init == 1)
                    if (false)
                    disp('Move back to home slowly - so no actuator starts acting up');
                    
                    timeTotal = 1; %s
                    steps = floor(timeTotal*50);
                    poses = linspace(500,0,steps)';
                    positions = repmat(poses,1,obj.serialInfo.numSegments);
                    
                    for k = 1:steps
                        obj.moveToPosition(positions(k,:),obj.serialInfo.noPreload)
                        pause(timeTotal/steps-0.005)
                    end
                    end
                    
                    zeroPosition = zeros(1,obj.serialInfo.numSegments);
                    obj.moveToPosition( zeroPosition,obj.serialInfo.noPreload );
                    
                    for i = 1:obj.serialInfo.numPorts
                        fclose(obj.serialInfo.serialPorts{i});
                        %display(['Closed Serial Port on ',obj.serialInfo.portName{i}])
                        delete(obj.serialInfo.serialPorts{i});
                        %display(['Deleted Serial Port on ',obj.serialInfo.portName{i}])
                        clear obj.serialInfo.serialPorts{i}
                    end
                    obj.serialInfo.init = 0;
                    %display('Closed+Deleted Serial Ports')
                    
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
                disp('[ForceControllerClient] Closing Serial Ports')
                obj.deinitSerial();
                disp('[ForceControllerClient] Closed Serial Ports')
            end
        end   
    end
    
    
    methods(Static)
        function [m1_cl, m1_ch, m2_cl, m2_ch] = posToMotorCmd(position,preload,maxPos,forceScaling,num)
            
            leftIdx = 1;
            rightIdx = 2;
            if(position <= 0.0)
                %negative positions
                position_setpoint = abs(position*forceScaling(leftIdx));
                if(maxPos(leftIdx)<position_setpoint)
                    disp(['[FC] #',num2str(num),...
                        '-Left at limit (',num2str(position_setpoint),'>',num2str(maxPos(leftIdx)),')']);
                end 
                position_setpoint = min(maxPos(leftIdx),position_setpoint);
                
                motor_1_cmd = uint16(preload(leftIdx) + position_setpoint);
                
                position_setpoint_neg = abs(1*position*forceScaling(rightIdx));
                %ensure the subtracted value is never lower than the
                %preload:
                position_setpoint_neg = min(preload(rightIdx),position_setpoint_neg);
                
                motor_2_cmd = uint16(preload(rightIdx) - position_setpoint_neg);
                
            else
                %positive positions
                position_setpoint = abs(position*forceScaling(rightIdx));
                if(maxPos(rightIdx)<position_setpoint)
                    disp(['[FC] ',num2str(num),...
                        '-Right at limit (',num2str(position_setpoint),'>',num2str(maxPos(rightIdx)),')']);
                end 
                position_setpoint = min(maxPos(rightIdx),position_setpoint);
                
                motor_2_cmd = uint16(preload(rightIdx) + position_setpoint);
                
                position_setpoint_neg = abs(1*position*forceScaling(leftIdx));
                %ensure the subtracted value is never lower than the
                %preload:
                position_setpoint_neg = min(preload(leftIdx),position_setpoint_neg);
                
                motor_1_cmd = uint16(preload(leftIdx) - position_setpoint_neg);
            end
            
            %% Prepare motor output
            m1_cl = uint8(uint16(192) + bitand(motor_1_cmd, uint16(31)));
            m1_ch = uint8(bitand(bitshift(motor_1_cmd, -5), uint16(127)));
            
            m2_cl = uint8(uint16(192) + bitand(motor_2_cmd, uint16(31)));
            m2_ch = uint8(bitand(bitshift(motor_2_cmd, -5), uint16(127)));
            
            
        end
        
    end
end
