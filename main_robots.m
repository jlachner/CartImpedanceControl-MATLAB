% Example script for Exp[licit], using the Cartesian Impedance controller
% Authors                       Email
%   [1] Johannes Lachner        jlachner@mit.edu
%   [2] Moses C. Nah            mosesnah@mit.edu
%

%% Cleaning up + Environment Setup
clear; close all; clc;

% Simulation settings
simTime = 12;           % Total simulation time
t  = 0;                 % The current time of simulation
dt = 0.01;              % Time-step of simulation

% CHOOSE YOUR ROBOT!
% robot = iiwa14( 'high' );
% robot = iiwa7( 'high' );
robot = franka;
robot.init( );

% Initialize robot parameters
q = robot.q_init;
dq = zeros( robot.nq, 1 );

%% Create animation
anim = Animation( 'Dimension', 3, 'xLim', [-0.7,0.7], 'yLim', [-0.7,0.7], 'zLim', [0,1.4] );
anim.init( );
anim.attachRobot( robot )

%% Controller initialization

% Kinematics and Dynamics
H_ee = robot.getForwardKinematics( q );
p_ee_ini = H_ee( 1:3, 4 );
R_ee_ini = H_ee( 1:3, 1:3 );
J_ee = robot.getHybridJacobian( q );
M = robot.getMassMatrix( q );

% Translational Cartesian Impedance Controller
K_p = 500 * eye( 3 );
impCtrl_p = CartImpController( 'translational' );
impCtrl_p.setStiffness( K_p );
impCtrl_p.setDampingRatio ( 0.7 );

% Rotational Cartesian Impedance Controller
K_e = 15 * eye( 3 );
impCtrl_e = CartImpController( 'rotational' );
impCtrl_e.setStiffness( K_e );
impCtrl_e.setDampingRatio( 0.7 );


%% Cyclic code starts here
while t <= simTime
    tic

    % ========================== %
    % ====== Get robot data ==== %
    % ========================== %

    % Get current robot transformation matrix of end-effector
    H_ee = robot.getForwardKinematics( q );
    R_ee = H_ee( 1:3, 1:3 );
    p_ee = H_ee( 1:3, 4 );

    % Get Hybrid Jacobian of a point on end-effector
    J_ee = robot.getHybridJacobian( q );

    % Get mass matrix of robot
    M = robot.getMassMatrix( q );
    M(7,7) = 25 * M(7,7);                           % Virtually increase the mass of the last joint

    % ============================ %
    % ======== Controller   ====== %
    % ============================ %

    % Get Trajectory
    A = 0.07;
    p_ee_0 = func_circularInterp( p_ee_ini, A , t, simTime/2, 2 );

    % End-effector control: Move around circular
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Update robot kinematics
    impCtrl_p.setKinematics( dq, H_ee, J_ee, M );                  
    impCtrl_p.setZFT( p_ee_0 );

    % Calculate elastic wrench and damping wrench
    F_p = impCtrl_p.getElasticWrench( );
    F_p_d = impCtrl_p.getDampingWrench( );

    % Map to joint torques
    tau_imp_p = J_ee' * ( F_p - F_p_d ); 

    % End-effector control: Hold initial orientation
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Update robot kinematics
    impCtrl_e.setKinematics( dq, H_ee, J_ee, M );
    impCtrl_e.setZFT( R_ee_ini );                              

    % Calculate elastic wrench and damping wrench
    F_e = impCtrl_e.getElasticWrench( );
    F_e_d = impCtrl_e.getDampingWrench( );

    % Map to joint torques
    tau_imp_e = J_ee' * ( F_e - F_e_d ); 

    % Add joint damping
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    B_q = 0.1;
    tau_q = - B_q * dq;

    % ============================ %
    % ====== Control torque ====== %
    % ============================ %

    tau = tau_imp_p + tau_imp_e + tau_q;


    % ============================================= %
    % ======== Proceed one simulation step ======== %
    % ============================================= %

    % Interpolation robot
    rhs = M \ tau;                                                          % We assume gravity and Coriolis are compensated
    [ q, dq ] = func_symplecticEuler( q, dq, rhs, dt );

    % Update the linkage plot
    robot.updateKinematics( q );

    % Update control time and counter
    t = t + dt;
    anim.update( t );

    % Do not go faster than real time
    while toc < dt
        % do nothing
    end
end

%% Close animation
anim.close();