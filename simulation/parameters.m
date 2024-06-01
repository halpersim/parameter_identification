% Load all parameters of KUKA lbr iiwa for simulink and simscape
% TW, 27.07.2020

g_factor = 1;

if is_manual_simulation()
    close all
    clear
    clc

    g_factor = -1;
    
    %% Set paths
    warning('off', 'MATLAB:rmpath:DirNotFound');
    rmpath(genpath('./kukalbriiwa_model/matlab'));
    warning('on','all');
    
    addpath('./kukalbriiwa_model/matlab/');
    addpath('./kukalbriiwa_model/meshes/');
    
    %the following two folders both contain the matrices of the equations of motion
    %however, in the first one, the matrices are expressed with respect to the mu paramter
    % and in the second one with respect to the pi_b paramter 
    %% ---- so which line is active decides which set of parameters is used
    addpath('./kukalbriiwa_model/matlab/withoutLinAxes/');
    %addpath('./kukalbriiwa_model/matlab/withoutLinAxes/dyn_parameter/');

    load("simulation_results.mat");
    mu = out.mu{1}.values.mu_act;
    %pi_b = pi_b;% param_out.pi.values.pi_b_ist;
else
    if evalin('base', 'exist("TEST_TRAJECTORY", "var")') == 1
        par.a = evalin('base', 'par.a');
        par.b = evalin('base', 'par.b');
        par.T = evalin('base', 'par.T');
        T = par.T;
    end

    if evalin('base', 'exist("G_FACTOR", "var")') == 1
        g_factor = evalin('base', 'G_FACTOR');
    end

    if evalin('base', 'exist("TEST_PI_B_PARAMETER", "var")') == 1
        pi_b = evalin('base', 'par.pi_b');
    end
    
    % if evalin('base', 'exist("TRAJECTORY_TYPE", "var")') == 1
    %     set_param('experiment_controller/Combo Box2', 'Value', num2str(evalin('base', 'TRAJECTORY_TYPE')));
    % end
end

load bus;


%% Parameters
if ~exist('T', 'var')
    par.T = 27;         % simulation time

    %% Sekunden pro trajektorie
    % 1: 17
    % 6: 40
    % 26:30
    % 13:31
    % 17:28
    % 7:35
    % 27:27
    load("trajectories.mat", 'erg');
    traj_idx = 27;
    par.a = erg{traj_idx}.traj.a;
    par.b = erg{traj_idx}.traj.b;
end

% set stop time of simulation to be the same as the length of the trajectory
set_param(gcs, 'StopTime', num2str(par.T));
par.Ta = 125e-6;    % sampling time

% Initial conditions
q_0 = zeros(7,1);
q_0_p = zeros(7,1);
q_0_pp = zeros(7,1);

%% Robot parameters KUKA lbr iiwa
kuka=true;

if kuka
    %Parameters of KUKA iiwa for Fachvertiefungs-coordinate system
    param_kuka_lab;

    param_robot.g = g_factor * param_robot.g;
    %param_robot.yaw = 0;
    if exist('mu', 'var')
        param_robot = load_mu(param_robot, mu);
    end
else
    %Parameters of simscape for Fachvertiefungs-coordinate system
    param_simscape;
end

if evalin('base', 'exist("TEST_MU_PARAMETER", "var")') == 1
    mu = evalin('base', 'par.mu');
    param_robot = load_mu(param_robot, mu);
end


%Parameters for simscape visualization
%if is_manual_simulation() 
if false
    param_visual;

    param_robot.R_0 = [[0, 1, 0]; [1, 0, 0]; [0, 0, -1]];
    param_robot.p_0 = transpose([0 0 0]);
    set_param('experiment_simulation/Visualisierung','Commented','off');
else
    set_param('experiment_simulation/Visualisierung','Commented','on');
end

%%Spline Trajektory preperation
%% insert the following code section into your file named "parameters.m"
%% Prepare data for Trajectory generator Spline
tmp=load('spline_test_trajectory.mat','q','time');
q=tmp.q;
time=tmp.time;

degree=3;
knot=time/time(end);
%Append and prepend the starting and ending knots
param_trajectory_generator_spline=struct();
param_trajectory_generator_spline.degree=degree;
param_trajectory_generator_spline.knot=[zeros(degree-1,1);knot;ones(degree-1,1)];
param_trajectory_generator_spline.position=[repmat(q(1,:),[degree-1,1]);...
  q;...
  repmat(q(end,:),[degree-1,1])];

function ret = is_manual_simulation()
    ret = evalin('base', 'exist("AUTO_SIM_FLAG", "var")') == 0;
end