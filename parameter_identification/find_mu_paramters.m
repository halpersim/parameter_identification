%% -------------- file can be ignored ---------------
% the purpose of this file is to iteratively find the best configuration by hand 
% for the fmincon solver to get good estimates for µ


param_in.my.fmincon_configs = get_my_fmincon_configs();

param_in.cut_first_n_elements = 20000/8;
param_in.filter.use_estimations = true;
param_in.load_data = false;
% traj_nr must be element of [1 6 26 13 17 7 27]
traj_nr = 26;

%last_traj_nr = -1;

cur_dir = pwd;
cd("..\simulation\kukalbriiwa_model\matlab\withoutLinAxes\");
param_kuka_lab;
param_in.param_robot = param_robot;
param_in.param_robot.g = -param_in.param_robot.g;
% param_in.param_robot.normal.pitch = 0;
% param_in.param_robot.normal.roll = pi;
% param_in.param_robot.normal.yaw = 0;
cd(cur_dir);

param_in.with_friction = true;
param_in.my.calculate = true;

param_in.filter.use_filter = true;
param_in.filter.fs = 1000;
param_in.filter.butter_order_q = 20;
param_in.filter.butter_order_tau = 20;

param_in.test.trajectory_name = "../parameter_identification/trajectories/strict_53.mat";
param_in.test.trajectory_duration = 20;
param_in.test.dyn_param_dir = "../simulation/kukalbriiwa_model/matlab/withoutLinAxes/dyn_parameter/";

trajectory_numbers = [1 6 26 13 17 7 27];

load("measurements_refined.mat");

if ~exist('measurements', 'var') 
    measurements = [];
    for traj_nr = trajectory_numbers
            [tau_meas, q_meas] = data_import(traj_nr);
            m.tau = tau_meas;
            m.q = q_meas;

            measurements = [measurements {m}];
    end
end

tau_meas = measurements{7}.tau;
q_meas = measurements{7}.q;

tau_meas_1k = tau_meas(1:8:end, :);
q_meas_1k = q_meas(1:8:end, :);

param_in.measurement.use = true;
param_in.measurement.tau = tau_meas_1k;
param_in.measurement.q = q_meas_1k;
param_in.measurement.qp = [];
param_in.measurement.qpp = [];

%4 - 4.3 (8.5/50.4)
%3.9 - 4.2 (8.5/50.4)
%3.4 - 3.6 (8.3/50.5)
%2.6 - 2.7 (7.9/51.7)
%2.5 - 2.6 (8.0/51.2)
%1.7 - 1.9 (13.2/55.4)

% q_cuttoffs = [1.7 2.5 2.6 3.4 3.9 4];
% tau_cutoffs = [1.9 2.6 2.7 3.6 4.2 4.3];
% q_cuttoffs = [2.5 2.6 3.4 3.9 4];
% tau_cutoffs = [2.6 2.7 3.6 4.2 4.3];

param_in.filter.fc_q = 2.5;
param_in.filter.fc_tau = 2.6;   

%results = [];
load("evaluation_different_paramter_02_23.mat");

param_out = identify_parameter(param_in);




function opt_ar = get_my_fmincon_configs()
    opt_ar = [];

    display = 'off';
    min_optimality = 1e+2;
    min_step = 1e-7;
    min_costraint = 1e-14;
    % min_optimality = 1e+2;
    % min_step = 1e-2;
    % min_costraint = 1e-6;
    fourier_coeffs = 10;
    traj_algorithm = 'interior-point';
    traj_duration = 20;


    f = optimoptions('fmincon', ...
        'Display','iter', ...
        'MaxFunctionEvaluations', 50000, ...
        'Algorithm', 'interior-point', ...
        'StepTolerance', 1e-7, ...
        'SpecifyObjectiveGradient',true, ...
        'ObjectiveLimit', 1e+2, ...
        'ConstraintTolerance', 1e-14, ...
        'EnableFeasibilityMode', true);
    opt_ar = [opt_ar {f}];

    % f = optimoptions('fmincon', ...
    %     'Display','iter', ...
    %     'MaxFunctionEvaluations', 50000, ...
    %     'Algorithm', 'interior-point', ...
    %     'StepTolerance', 1e-7, ...
    %     'SpecifyObjectiveGradient',false, ...
    %     'ObjectiveLimit', 1e+2, ...
    %     'ConstraintTolerance', 1e-14, ...
    %     'EnableFeasibilityMode', true);
    % opt_ar = [opt_ar {f}];
    % 
    f = optimoptions('fmincon', ...
        'Display','iter', ...
        'MaxFunctionEvaluations', 50000, ...
        'Algorithm', 'sqp', ...
        'StepTolerance', 1e-2, ...
        'SpecifyObjectiveGradient',true, ...
        'ObjectiveLimit', 1e+2, ...
        'ConstraintTolerance', 1e-6, ...
        'EnableFeasibilityMode', true);
    % 
    % opt_ar = [opt_ar {f}];
    %     f = optimoptions('fmincon', ...
    %     'Display','off', ...
    %     'MaxFunctionEvaluations', 50000, ...
    %     'Algorithm', 'sqp', ...
    %     'StepTolerance', 1e-2, ...
    %     'SpecifyObjectiveGradient',false, ...
    %     'ObjectiveLimit', 1e+2, ...
    %     'ConstraintTolerance', 1e-6, ...
    %     'EnableFeasibilityMode', true);
    % opt_ar = [opt_ar {f}];
end 

function [a,b] = extract_ab(x, coeff_len) 
    a = zeros(7,coeff_len);
    b = zeros(7,coeff_len);

    for i = 1:7
        a(i, :) = x(1+(i-1)*coeff_len : i*coeff_len);
        b(i, :) = x(coeff_len*7 + 1+(i-1)*coeff_len : coeff_len*7 + i*coeff_len);
    end
end