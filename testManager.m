%% run experiment

%% Curvature Level Controller
setup = struct();
%setup.expType = ExpTypes.OptitrackOnly;
setup.expType = ExpTypes.GelSightOnly;
setup.expTimeSeconds = 10; %70
setup.framePeriod = 1;
setup.offset = [0,0,0];
setup.camNum=2;
setup.timestep = 0.01;
setup.debug = false;

%generate manager object
manager = Manager(setup);

manager.start();

if(manager.isvalid == 1)
    delete(manager);
end
