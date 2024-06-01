% Inputs:
% in    (structure)
%   in.with_friction                    (bool)                   - consider friction
%   in.param_robot                      (structure)              - structure containing the robots parameter
%   in.measurement[.tau .q .qp .qpp]    (n x 7 - double)         - measurements that are the basis for the parameter identification
%   in.cut_first_n_elements             (int)                    - number of how many values of the filtered values of q and tau should be discarded (this is useful, because the measurements practically stay the same for the first ~x elements (see data_import))
%
%   in.filter               (structure)                          - filter configuration
%   	in.filter.use_estimations       (bool)                   - if qp and qpp should be estimated from q
%       in.filter.use_filter            (bool)                   - if the measurements should be filtered
%       in.filter.fs                    (double)                 - sample frequency of the measurements
%       in.filter.fc_q                  (double)                 - cutoff frequency of the low-pass filter for q
%       in.filter.fc_tau                (double)                 - cutoff frequency of the low-pass filter for tau
%       in.filter.butter_order_q        (int)                    - order of the low-pass filter for q
%       in.filter.butter_order_tau      (int)                    - order of the low-pass filter for tau
%
%   in.mu                   (structure)                          - mu configuration
%       in.mu.calculate                 (bool)                   - if mu should be calculated
%       in.mu.fmincon_configs           (n x 1 - optimoptions)   - options that will be given to fmincon to calculate mu
%
%   in.test                 (structure)
%       in.test.trajectory_name         (string)                 - name of the file of trajectory with which the pi parameters will be tested
%       in.test.trajectory_duration     (double)                 - length of the test trajectory
%       in.test.dyn_param_dir           (string)                 - directory where the matlab files discribing the equations of motion are (inertia_matrix.m, coriolis_matrix.m, ...) 

clear all;

cur_dir = pwd;
cd("..\simulation\kukalbriiwa_model\matlab\withoutLinAxes\");
param_kuka_lab;
in.param_robot = param_robot;
in.param_robot.g = -in.param_robot.g;
cd(cur_dir);

load("export_simulation.mat");
in.measurement.q = squeeze(data{1}.Values.Data);
in.measurement.qp = 0;
in.measurement.qpp = 0;
in.measurement.tau = squeeze(data{2}.Values.Data);

in.with_friction = false;

in.cut_first_n_elements = 1;

in.filter.use_filter = false;
in.filter.use_estimations = true;
in.filter.fs = 100;

in.mu.calculate = true;
in.mu.fmincon_configs = {optimoptions('fmincon', ...
        'Display','off', ...
        'MaxFunctionEvaluations', 50000, ...
        'Algorithm', 'interior-point', ...
        'StepTolerance', 1e-7, ...
        'SpecifyObjectiveGradient',true, ...
        'checkGradients', false, ...
        'ObjectiveLimit', 1e+2, ...
        'ConstraintTolerance', 1e-14, ...
        'EnableFeasibilityMode', false)};

in.test.trajectory_name = "../parameter_identification/trajectories/strict_53.mat";
in.test.trajectory_duration = 20;
in.test.dyn_param_dir = "../simulation/kukalbriiwa_model/matlab/withoutLinAxes/dyn_parameter/";

out = identify_parameter(in);

pi_b = out.pi.values.pi_b_act;
mu = out.mu{1}.values.mu_act;
mu_exp = out.mu{1}.values.mu_exp;

