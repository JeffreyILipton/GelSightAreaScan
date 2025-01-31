% Robotics Toolbox.
% Version 10.1.0  2017-10-23
%
%
% Homogeneous transformations 3D
%    angvec2r                   - angle/vector to RM
%    angvec2tr                  - angle/vector to HT
%    eul2r                      - Euler angles to RM
%    eul2tr                     - Euler angles to HT
%    ishomog                    - true if argument is a 4x4 matrix
%    isunit                     - true if argument is a unit vector
%    isrot                      - true if argument is a 3x3 matrix
%    oa2r                       - orientation and approach vector to RM
%    oa2tr                      - orientation and approach vector to HT
%    rotx                       - RM for rotation about X-axis
%    roty                       - RM for rotation about Y-axis
%    rotz                       - RM for rotation about Z-axis
%    rpy2r                      - roll/pitch/yaw angles to RM
%    rpy2tr                     - roll/pitch/yaw angles to HT
%    tr2angvec                  - HT/RM to angle/vector form
%    tr2eul                     - HT/RM to Euler angles
%    tr2rpy                     - HT/RM to roll/pitch/yaw angles
%    transl                     - set or extract the translational component of HT
%    trchain                    - sequence of HT
%    trinterp                   - interpolate an HT
%    tripleangle                - GUI for triple angle visualization
%    trnorm                     - normalize HT
%    trprint                    - print an HT
%    trotx                      - HT for rotation about X-axis
%    troty                      - HT for rotation about Y-axis
%    trotz                      - HT for rotation about Z-axis
%    trlog                      - efficient logarithm
%    trexp                      - efficient exponentiation
%    trscale                    - pure scale change
%
% Homogeneous transformations 2D
%    ishomog2                   - true if argument is a 4x4 matrix
%    isrot2                     - true if argument is a 3x3 matrix
%    rot2                       - rotation matrix in 2D
%    transl2                    - set or extract the translational component of HT
%    trchain2                   - sequence of HT
%    trexp2                     - exp in 2D
%    trinterp2                  - interpolate an HT
%    trot2                      - HT for rotation in 2D
%    trprint2                   - print an HT
%
% Homogeneous transformation utilities
%    r2t                        - RM to HT
%    t2r                        - HT to RM
%    rt2tr                      - (R,t) to HT
%    tr2rt                      - HT to (R,t)
%
% Homogeneous points and lines
%    e2h                        - Euclidean coordinates to homogeneous
%    h2e                        - homogeneous coordinates to Euclidean
%    homline                    - create line from 2 points
%    homtrans                   - transform points
%
%  * HT = homogeneous transformation, a 4x4 matrix, belongs to the group SE(3).
%  * RM = RM, an orthonormal 3x3 matrix, belongs to the group SO(3).
%  * Functions of the form <b>tr2XX</b> will also accept a RM as the argument.
%
% Differential motion
%    delta2tr                   - differential motion vector to HT
%    eul2jac                    - Euler angles to Jacobian
%    rpy2jac                    - RPY angles to Jacobian
%    skew                       - vector to skew symmetric matrix
%    skewa                      - vector to augmented skew symmetric matrix
%    tr2delta                   - HT to differential motion vector
%    tr2jac                     - HT to Jacobian
%    vex                        - skew symmetric matrix to vector
%    vexa                       - augmented skew symmetric matrix to vector
%    wtrans                     - transform wrench between frames
%
% Trajectory generation
%    ctraj                      - Cartesian trajectory
%    jtraj                      - joint space trajectory
%    lspb                       - 1D trapezoidal trajectory
%    mtraj                      - multi-axis trajectory for arbitrary function
%    mstraj                     - multi-axis multi-segment trajectory
%    tpoly                      - 1D polynomial trajectory
%    trinterp                   - interpolate HT s
%    trinterp2                  - interpolate HT s
%
% Pose representation classes
%    +ETS2                      - elementary transform sequence in 2D
%    +ETS3                      - elementary transform sequence in 3D
%    Quaternion                 - general quaternions
%    SO2                        - rotations SO(2)
%    SE2                        - poses SE(2)
%    SO3                        - rotations SO(3)
%    SE3                        - poses SE(3)
%    Twist                      - 2D and 3D twists
%    UnitQuaternion             - unit quaternions
%    RTBPose                    - Abstract pose support
%
% Serial-link manipulator
%    DHFactor                   - convert elementary transformations to DH form
%    Link                       - construct a robot link object
%    Prismatic                  - construct a prismatic link object
%    PrismaticMDH               - construct a prismatic link object (MDH params)
%    Revolute                   - construct a revolute link object
%    RevoluteMDH                - construct a revolute link object (MDH params)
%    SerialLink                 - construct a serial-link robot object
%    friction                   - return joint friction torques
%    nofriction                 - return a robot object with no friction
%    perturb                    - return a robot object with perturbed parameters
%    plot                       - plot/animate robot
%    teach                      - drive a graphical  robot
%
%     Models
%        models                 - list/search all models
%        mdl_KR5                - Kuka KR5
%        mdl_S4ABB2p8           - ABB S4 2.8 (DH, kine)
%        mdl_ball               - high DOF ball robot
%        mdl_baxter             - Rethink Robotics Baxter
%        mdl_cobra600           - Adept SCARA robot
%        mdl_coil               - high DOF coiled robot
%        mdl_Fanuc10L           - Fanuc 10L (DH, kine)
%        mdl_hyper2d            - high DOF planar
%        mdl_hyper3d            - high DOF 3D
%        mdl_irb140             - ABB IRB140
%        mdl_irb140_mdh         - ABB IRB140 (MDH)
%        mdl_jaco               - Kinova Jaco (DH)
%        mdl_KR5                - Kuka KR5  (DH)
%        mdl_LWR                - Kuka LWR (DH)
%        mdl_m16                - Fanuc M16
%        mdl_mico               - Kinova Mico (DH)
%        mdl_MotomanHP6         - Motoman HP6 (DH, kine)
%        mdl_nao                - Alderbaran Nao humanoid
%        mdl_p8                 - Puma 560 on an XY base
%        mdl_phantomx           - PhantomX pincher
%        mdl_planar1            - 1 line planar robot
%        mdl_planar2            - 1 line planar robot
%        mdl_planar3            - 1 line planar robot
%        mdl_puma560            - Puma 560 data (DH, kine, dyn)
%        mdl_puma560akb         - Puma 560 data (MDH, kine, dyn)
%        mdl_quadrotor          - simple quadcopter model
%        mdl_stanford           - Stanford arm data (DH, kine, dyn)
%        mdl_stanford_mdh       - Stanford arm data (MDH, kine)
%        mdl_twolink            - simple 2-link example (DH, kine)
%        mdl_twolink_mdh        - simple 2-link example (MDH, kine)
%        mdl_twolink_sym        - simple 2-link example (DH, kine, symbolic)
%        mdl_ur3                - Universal Robotics UR3 (DH, kine, dyn)
%        mdl_ur5                - Universal Robotics UR5 (DH, kine, dyn)
%        mdl_ur10               - Universal Robotics UR10 (DH, kine, dyn)
%
%     Kinematics
%        DHFactor               - transform sequence to DH description
%        ETS2                   - simplified kinematics in 2D
%        ETS3                   - simplified kinematics in 3D
%        jsingu                 - find joint singularity
%        fkine                  - forward kinematics
%        ikine                  - inverse kinematics (numeric)
%        ikine6s                - inverse kinematics for 6-axis arm with sph.wrist
%        jacob0                 - Jacobian in base coordinate frame
%        jacobe                 - Jacobian in end-effector coordinate frame
%        maniplty               - compute manipulability
%        trchain                - transform chain in 3D
%        trchain2               - transform chain in 2D
%
%     Dynamics
%        accel                  - forward dynamics
%        cinertia               - Cartesian manipulator inertia matrix
%        coriolis               - centripetal/coriolis torque
%        fdyn                   - forward dynamics
%        wtrans                 - transform a force/moment
%        gravload               - gravity loading
%        inertia                - manipulator inertia matrix
%        itorque                - inertia torque
%        rne                    - inverse dynamics
%
% Mobile robot
%    LandmarkMap                - point feature map object
%    RandomPath                 - driver for Vehicle object
%    RangeBearingSensor         - "laser scanner" object
%    Vehicle                    - abstract vehicle superclass
%    Bicycle                    - construct a mobile robot with bicycle like kinematics
%    Unicycle                   - construct a mobile robot with unicycle like kinematics
%    sl_bicycle                 - Simulink "bicycle model" of non-holonomic wheeled vehicle
%    Navigation                 - Navigation superclass
%    Sensor                     - robot sensor superclass
%    plot_vehicle               - plot vehicle
%
%     Localization
%        EKF                    - extended Kalman filter object
%        ParticleFilter         - Monte Carlo estimator
%        PoseGraph              - pose-graph SLAM
%
%     Path planning
%        Bug2                   - bug navigation
%        DXform                 - distance transform from map
%        Dstar                  - D* planner
%        Lattice                - lattice planner
%        PRM                    - probabilistic roadmap planner
%        RRT                    - rapidly exploring random tree
%
% Graphics
%    circle                     - compute/draw points on a circle
%    mplot                      - time series plotting
%    plot2                      - plot trajectory
%    plotp                      - plot points
%    plotvol                    - set plot bounds
%    plot_arrow                 - draw an arrow
%    plot_box                   - draw a box
%    plot_circle                - draw a circle
%    plot_ellipse               - draw an ellipse
%    plot_homline               - plot homogeneous line
%    plot_point                 - plot points
%    plot_poly                  - plot polygon
%    plot_sphere                - draw a sphere
%    qplot                      - plot joint angle trajectories
%    trplot                     - plot HT as a coordinate frame
%    trplot2                    - plot HT, SE(2), as a coordinate frame
%    tranimate                  - animate a coordinate frame
%    tranimate2                 - animate a coordinate frame
%    xaxis                      - set x-axis scaling
%    yaxis                      - set y-axis scaling
%    xyzlabel                   - label axes x, y and z
%
% Utility
%    about                      - summary of object size and type
%    angdiff                    - subtract 2 angles modulo 2pi
%    bresenham                  - Bresenhan line drawing
%    colnorm                    - columnwise norm of matrix
%    diff2                      - elementwise diff
%    distancexform              - compute distance transform
%    edgelist                   - trace edge of a blob
%    gauss2d                    - Gaussian distribution in 2D
%    ismatrix                   - true if non scalar
%    isvec                      - true if argument is a 3-vector
%    isunit                     - true if argument is a unit vector
%    numcols                    - number of columns in matrix
%    numrows                    - number of rows in matrix
%    peak                       - find peak in 1D signal
%    peak2                      - find peak in 2D signal
%    PGraph                     - general purpose graph class
%    pickregion                 - pick a region in figure
%    Plucker                    - Plucker line class
%    polydiff                   - derivative of polynomial
%    Polygon                    - general purpose polygon class
%    randinit                   - initialize random number generator
%    ramp                       - create linear ramp
%    runscript                  - step through a script file
%    unit                       - unitize a vector
%    tb_optparse                - toolbox argument parser
%    stlRead                    - read an STL file
%    RTBPlot                    - graphics support
%    chi2inv_rtb                - simple inverse chi squared
%
% Demonstrations
%    rtbdemo                    - Serial-link manipulator demonstration
%
% Examples
%    sl_quadcopter              - Simulink model of a flying quadcopter
%    sl_braitenberg             - Simulink model a Braitenberg vehicle
%    movepoint                  - non-holonomic vehicle moving to a point
%    moveline                   - non-holonomic vehicle moving to a line
%    movepose                   - non-holonomic vehicle moving to a pose
%    walking                    - example of 4-legged walking robot
%    eg_inertia                 - joint 1 inertia I(q1,q2)
%    eg_inertia22               - joint 2 inertia I(q3)
%    eg_grav                    - joint 2 gravity load g(q2,q3)
%
%  *  located in the examples folder
%
% Copyright (C) 2011 Peter Corke
