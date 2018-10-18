%% setup experiment enviroment
clearvars -EXCEPT hwObj
close all
clc
%%
useHardware = true;

if ~exist('hwObj') && useHardware
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
    
    hwObj = UR;
end

%% Curvature Level Controller
setup = struct();
%setup.expType = ExpTypes.OptitrackOnly;
%setup.expType = ExpTypes.GelSightOnly;
setup.expType = ExpTypes.TestArm;
setup.expTimeSeconds = 0;%10; %70
setup.framePeriod = 1;
setup.offset = [0,0,0];
setup.camNum=2;
setup.timestep = 0.01;
setup.debug = false;%true;%


setup.HSA_mins = [1300,1450,1360,1300];
setup.HSA_maxs = [1140,1600,1200,1500];
setup.HSA_channels = 0:3;
setup.HSA_port = 'COM4';

setup.useHardware = useHardware;
setup.origin = [0.0,0.0,0.0];
setup.xysize = [250.0,250.0];
setup.delta = 10.0;
if exist('hwObj')
    setup.hwObj = hwObj;
end
%generate manager object
manager = Manager(setup);

manager.start();

if(manager.isvalid == 1)
    delete(manager);
end
