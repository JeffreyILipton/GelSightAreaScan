%% run experiment

%% Curvature Level Controller
setup = struct();
setup.expType = ExpTypes.PhysicalExperiment;
%setup.expType = ExpTypes.Characterization;

setup.expTimeSeconds = 10; %70
setup.armDof = 5;
setup.gripperExists = false;
setup.amplitude = 10/180*pi;
setup.numPeriods = 4;
setup.offset = 10/180*pi;
setup.profile = '-cos';
setup.enable_PID = 0;
setup.kp = 0.05;
setup.kd = 0;
setup.ki = 0;


setup.actuatorActive = true;
setup.controllerType = ControllerTypes.CurvatureLevelControl;

%setup.controllerTypeInput = CartesianImpedanceControl;

setup.nyquistConstant = 5;

%generate manager object
manager = Manager(setup);

manager.start();

if(manager.isvalid == 1)
    delete(manager);
end
