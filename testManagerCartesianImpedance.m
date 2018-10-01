%% run experiment
clear all

%% Curvature Level Controller
setup = struct();
setup.expType = ExpTypes.PhysicalExperiment;
%setup.expType = ExpTypes.Characterization;
setup.numContactForces = 3;
setup.framePeriod = 0.0146;

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
%setup.controllerType = ControllerTypes.CurvatureLevelControl;
setup.controllerType = ControllerTypes.CartesianImpedanceControl;

setup.ci_Kp = 14*eye(3);
setup.ci_Kd = 5*eye(3);

setup.rootPose = [0.102,0.145,0];
setup.cartPoseDes = [0.368,0.288,0] - setup.rootPose;
%setup.cartPoseDes = [0.366,0.284,0] - setup.rootPose;
%setup.cartPoseDes = [0.394, 0.235,0] - setup.rootPose;
setup.phaseOneCartPoseDes = [0.368,0.288,0] - setup.rootPose; %Put here a point INSIDE the environment. By trying to reach it the robot would impact the environment surface.
%setup.phaseTwoCartPoseDes = [0.368,0.288,0] - setup.rootPose; %Put here a point ON the environment surface. This is the desired steady state end effector position.


setup.nyquistConstant = 7;
% force threshold for contact detaction
setup.forceThreshold = 100;

manager = Manager(setup);

manager.start();

if(manager.isvalid == 1)
    delete(manager);
end
