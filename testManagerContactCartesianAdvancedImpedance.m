%% run experiment
clear all
%close all

%% Complex Contact Controller
setup = struct();
setup.expType = ExpTypes.PhysicalExperimentEnvironment;
setup.numContactForces = 3;
setup.framePeriod = 0.0186;%0.045269294761075;%0.0186;

setup.expTimeSeconds = 10; %10
setup.armDof = 6;
setup.gripperExists = false;
%setup.amplitude = 10/180*pi;
%setup.numPeriods = 4;
%setup.offset = 10/180*pi;
%setup.profile = '-cos';
%setup.enable_PID = 0;
%setup.kp = 0.05;
%setup.kd = 0;
%setup.ki = 0;

setup.actuatorActive = true; %default = true
%setup.controllerType = ControllerTypes.CurvatureLevelControl;
setup.controllerType = ControllerTypes.CartesianImpedanceAdvancedControl;

% CART IMPE
%setup.ci_Kp = .75*1.1*eye(3);%13
%setup.ci_Kd = .25*eye(3); %6

%setup.ci_Kp = .65*1.1*eye(3);%13
%setup.ci_Kd = .30*eye(3); %6

setup.ci_Kp = .52*1.1*eye(3);%.55
setup.ci_Kd = .32*eye(3); %0.3


setup.ci_Ki = 0.0; %1.9 previously

setup.rootPose = [0.02683,0.1436,0]; % CHEKC FIRST WITH MOTIVE
%setup.cartPoseDes = [0.368,0.288,0] - setup.rootPose;


%[0.307,0.327,0]
%[0.300,0.367,0]
%setup.phaseOneCartPoseDes = [0.387,0.280,0.0] - setup.rootPose; %Put here a point INSIDE the environment. By trying to reach it the robot would impact the environment surface.
% TODO: Set up this second value properly:
%setup.phaseTwoCartPoseDes = [0.264,0.376,0.0] - setup.rootPose; %Put here a point ON the environment surface. This is the desired steady state end effector position.

%POSE 2
%setup.phaseOneCartPoseDes = [0.403,0.220,0.0] - setup.rootPose;
%setup.phaseTwoCartPoseDes = [0.319,0.356,0.0] - setup.rootPose;

% POSE 3
%setup.phaseOneCartPoseDes = [0.400,0.288,0.0] - setup.rootPose;
%setup.phaseTwoCartPoseDes = [0.267,0.405,0.0] - setup.rootPose;

% POSE 4 (flipped+
%setup.phaseOneCartPoseDes = [0.405,0.242,0.0] - setup.rootPose;
%setup.phaseTwoCartPoseDes = [0.270,0.328,0.0] - setup.rootPose;

% POSE 5 (long environment)
setup.phaseOneCartPoseDes = [0.43635,0.24958,0.0] - setup.rootPose;
setup.phaseTwoCartPoseDes = [0.250,0.441,0.0] - setup.rootPose;

% ENVIRONMENT RELATED

setup.environmentType = EnvironmentTypes.Sinusoid;
setup.plotting = false; %default = false
% desired "penetration" into the contact surface
setup.delta_ort = 0.03; %0.07
%n_par = ([313,314,0]-[373,257,0]);
%n_par = n_par/norm(n_par);
%setup.n_par = n_par; 

%setup.n_ort = [n_par(1),-n_par(2),0];
% force threshold for contact detection
%setup.forceThreshold = 100;

%direction of contact force measurementes
%obj.setup.meas_dir(:,1) = [1 0];
%obj.setup.meas_dir(:,2) = [1 1]./sqrt(2);
%obj.setup.meas_dir(:,3) = [0 1];

setup.nyquistConstant = 5;

% Low pass filter for velocity
setup.filter_cut_off_frequency = 30;

%% START
manager = Manager(setup);

manager.start();

if(manager.isvalid == 1)
    delete(manager);
end
