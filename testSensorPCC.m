%Create a curvature controller

setup = struct();
setup.expType = ExpTypes.Sensing;
%setup.expType = ExpTypes.Characterization;
setup.numContactForces = 3;
setup.framePeriod = 0.04;%0.018;%0.04;

setup.expTimeSeconds = 4; %70
setup.armDof = 6;
setup.gripperExists = false;
setup.amplitude =  deg2rad(10);%9/180*pi; % varied this between 7,10,13
setup.numPeriods = 8; %varied those between 6, 8, 9, 10
setup.offset = setup.amplitude;% + 0*9/180*pi;
setup.profile = '-cos';
%other trajectory options:
%setup.profile = 's_shape';
%setup.profile = '4_tip';

setup.enable_PID = 0;
setup.kp = 0;
setup.kd = 0;
setup.ki = 0.01;


setup.actuatorActive = true;
setup.controllerType = ControllerTypes.CurvatureLevelControl;

%setup.controllerTypeInput = CartesianImpedanceControl;

setup.nyquistConstant = 5;
% Low pass filter for velocity
setup.filter_cut_off_frequency = 15;

manager = Manager(setup);

manager.start();

manager.delete()