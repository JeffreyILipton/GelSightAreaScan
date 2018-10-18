%% run experiment

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

setup.useHardware = true; %false;
setup.origin = [0.0,0.0,0.0];
setup.xysize = [250.0,250.0];
setup.delta = 10.0;
%generate manager object
manager = Manager(setup);

manager.start();

if(manager.isvalid == 1)
    delete(manager);
end
