%% run experiment
clear all

%% Curvature Level Controller
setup = struct();
setup.expType = ExpTypes.PhysicalExperiment;
%setup.expType = ExpTypes.Characterization;
setup.numContactForces = 3;
setup.framePeriod = 0.0137;

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

% CART IMPE
setup.ci_Kp = 13*eye(3);
setup.ci_Kd = 6*eye(3);
setup.ci_Ki = 1.9;

setup.rootPose = [0.102,0.145,0];
setup.cartPoseDes = [0.368,0.288,0] - setup.rootPose;
%setup.cartPoseDes = [0.394, 0.235,0] - setup.rootPose;


%
setup.phaseOneCartPoseDes = [0.385,0.28,0] - setup.rootPose; %Put here a point INSIDE the environment. By trying to reach it the robot would impact the environment surface.
%setup.phaseOneCartPoseDes = [0.356,0.295,0] - setup.rootPose; %Put here a point INSIDE the environment. By trying to reach it the robot would impact the environment surface.
setup.phaseTwoCartPoseDes = [0.322,0.306,0] - setup.rootPose; %Put here a point ON the environment surface. This is the desired steady state end effector position.
%setup.phaseTwoCartPoseDes = [0.280,0.296,0] - setup.rootPose; %Put here a point ON the environment surface. This is the desired steady state end effector position.
% desired "penetration" into the contact surface
setup.delta_ort = 0.05;
n_par = ([313,314,0]-[373,257,0]);
n_par = n_par/norm(n_par);
setup.n_par = n_par; 

setup.n_ort = [n_par(1),-n_par(2),0];
% force threshold for contact detection
setup.forceThreshold = 100;

%direction of contact force measuramentes
obj.setup.meas_dir(:,1) = [1 0];
obj.setup.meas_dir(:,2) = [1 1]./sqrt(2);
obj.setup.meas_dir(:,3) = [0 1];

setup.nyquistConstant = 5;

%% START
manager = Manager(setup);

manager.start();

if(manager.isvalid == 1)
    delete(manager);
end
