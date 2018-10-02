%% run experiment

%% Curvature Level Controller
setup = struct();
setup.expType = ExpTypes.OptitrackOnly;

setup.expTimeSeconds = 10; %70
setup.framePeriod = 1;
setup.offset = [0,0,0];


%generate manager object
manager = Manager(setup);

manager.start();

if(manager.isvalid == 1)
    delete(manager);
end
