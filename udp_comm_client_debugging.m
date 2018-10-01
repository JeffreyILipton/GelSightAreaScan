% Run this on the client computer that has all the pumps connected to it
function [currentTime1,currentTime2,commandedValues,counting] = udp_comm_client()
cleanupObj = onCleanup(@cleanMeUp);
pause(1)
armdof = 6;
cleanedUp = false;
timeToRun = 10; %seconds
%host
%clientip = '128.30.25.105';
hostip = '128.30.25.104';
clientport = 8866;
hostport = 8844;
%u1 = udp(clientip, 'RemotePort',clientport , 'LocalPort', hostport);
u2 = udp(hostip, 'RemotePort',hostport , 'LocalPort', clientport,'timeout',1000,'DatagramTerminateMode','on');
u2.InputBufferSize = floor(armdof*2); % fix this to the length of the arm, so new messages overwrite old state
notOpen = true;
while(notOpen)
    try
        fopen(u2);
        notOpen = false;
    catch ME
        warning('UDP port already opened, need to close first!')
        %instrfind
        instrreset
        pause(1)
    end
end
%
% DlgH = figure;
% H = uicontrol('Style', 'PushButton', ...
%     'String', 'Break', ...
%     'Callback', 'delete(gcbf)');

pause(1);

%fc = ForceControllerClient(armdof,true);
%figure
%hold on
try
    disp('Waiting for messages!');
	%Clear buffer in case there is something in it
    if(u2.bytesAvailable>0)
        fread(u2,u2.bytesAvailable,'int16');
    end
	t1 = tic;
    memory = 10000;
    currentTime1 = zeros(memory,1);
    currentTime2 = zeros(memory,1);
    commandedValues = zeros(memory,1);
    counting = 1;
    actualTime = 0;
    while(actualTime<timeToRun)% && ishandle(H))
        [command,count] = fread(u2,armdof,'int16');
        actualTime = toc(t1);
        currentTime1(counting) = actualTime;
        commandedValues(counting) = command(3);
        if isempty(command)
            disp('timeout!');
        elseif count ~=armdof
            u2.BytesAvailable
            disp(['Wrong length: ',num2str(length(command))]);
            disp(num2str(command')); 
            disp(msg);
        else
            %disp('Received command: ');
            %disp(num2str(command'));
            %fc.sendPositions(command);
        end
        %fwrite(u2,command);
        currentTime2(counting) = toc(t1);
        counting = counting+1;
        %if(counting > memory)
                   
        %plot(currentTime1,commandedValues,'b');
        %plot(currentTime2,commandedValues,'r');
        %counting = 0;

        %end
        
    end
        

catch
    
    closeItAll();
    return
end
closeItAll();


    function closeItAll()
        %fc.delete()
        %clear fc
        if(~cleanedUp)
        fclose(u2);
        delete(u2)
        clear u2
        cleanedUp = true;
        end
    end
% fires when main function terminates
    function cleanMeUp()
        closeItAll()
    end
end
