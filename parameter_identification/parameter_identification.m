%this file is the usual startpoint for different operations 
%therefore it is not very structured or clean
% e. g. some variables might be redundant or useless and there is a lot of commented code

% currently it loads the measurements of trajectory 27 (the one which is used for paramter identification)
% calculates the paramters with different filters and simulates the results

param_in.mu.fmincon_configs = get_mu_fmincon_configs();

param_in.cut_first_n_elements = 20000/8;
%param_in.cut_first_n_elements = 1263;
param_in.filter.use_estimations = true;
param_in.load_data = false;

cur_dir = pwd;
cd("..\simulation\kukalbriiwa_model\matlab\withoutLinAxes\");
param_kuka_lab;
param_in.param_robot = param_robot;
param_in.param_robot.g = -param_in.param_robot.g;
cd(cur_dir);

param_in.with_friction = true;
param_in.mu.calculate = false;

param_in.filter.use_filter = true;
param_in.filter.fs = 1000;
param_in.filter.fc_q = 1.7;
param_in.filter.fc_tau = 2.1;
param_in.filter.butter_order_q = 20;
param_in.filter.butter_order_tau = 20;

param_in.test.trajectory_name = "../parameter_identification/trajectories/strict_53.mat";
param_in.test.trajectory_duration = 20;
param_in.test.dyn_param_dir = "../simulation/kukalbriiwa_model/matlab/withoutLinAxes/dyn_parameter/";

trajectory_numbers = [1 6 26 13 17 7 27];

load("results/measurements_averaged_reduced.mat");

if ~exist('measurements', 'var') 
    measurements = [];
    for traj_nr = trajectory_numbers
        [tau_meas, q_meas] = data_import(traj_nr);
        m.tau = tau_meas;
        m.q = q_meas;

        measurements = [measurements {m}];
    end
end

tau_meas = measurements{4}.tau;
q_meas = measurements{4}.q;

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

% q_cuttoffs = [2.5 2.6 3.4 3.9 4];
% tau_cutoffs = [2.6 2.7 3.6 4.2 4.3];

% q_cuttoffs = [1.7 2 2.5 2.6];
% tau_cutoffs = [1.9 2.1 2.6 2.7];

q_cuttoffs = [2];
tau_cutoffs = [2.2];

results = [];
%load("evaluation_different_paramter_02_23.mat");

% for i = 1:max(size(q_cuttoffs))
%     q_cutoff = q_cuttoffs(i);
%     tau_cutoff = tau_cutoffs(i);
% 
%     param_in.filter.fc_q = q_cutoff;
%     param_in.filter.fc_tau = tau_cutoff;   
% 
%     param_out = identify_parameter(param_in);
% 
%     for traj_nr = [7 27]
%         mu_res = [];
%         disp("trajectory nr: " + num2str(traj_nr));
% 
%         % for l = 1:max(size(param_out.mu))
%         %     disp("start simulating mu [" + num2str(q_cutoff) + ", " + num2str(tau_cutoff) + "]");
%         %     tic; 
%         %     [tau_sim, q_sim] = simulate_parameter(param_out.mu{l}.values.mu_act, "mu", traj_nr);
%         %     disp("end simulation - duration = " + num2str(toc)); 
%         % 
%         %     mu_l.tau = tau_sim;
%         %     mu_l.q = q_sim;
%         % 
%         %     mu_res = [mu_res {mu_l}];
%         % end
% 
%         disp("start simulating pi_b [" + num2str(q_cutoff) + ", " + num2str(tau_cutoff) + "]");
%         tic; 
%         [tau_sim, q_sim] = simulate_parameter(param_out.pi.values.pi_b_act, "pi_b", traj_nr);
%         disp("end simulation - duration = " + num2str(toc)); 
% 
%         pi_res.tau = tau_sim;
%         pi_res.q = q_sim;
% 
%         res = [];
% 
%         res.pi = pi_res;
%         res.mu = mu_res;
%         res.fc_q = q_cutoff;
%         res.fc_tau = tau_cutoff;
%         res.traj_nr = traj_nr;      
%         res.params = param_out;
% 
%         results = [results {res}];
%         save("complete_evaluation_02_27.mat", 'results');
%     end 
% end


for causal = 1:2
    for q_cutoff = 1.3:0.1:3
        for tau_cutoff = (q_cutoff - 0.2):0.1:(q_cutoff + 0.5)
            param_in.filter.fc_q = q_cutoff;
            param_in.filter.fc_tau = tau_cutoff;   
            param_in.filter.non_causal_filter = causal == 1;

            param_out = identify_parameter(param_in);
    
            if param_out.pi.feasible == "inertia matrix positive definite (100%) - skew symmetry of Bt - 2C (100%)"
                % disp("cuttoff: q = " + num2str(q_cutoff) + ", tau = "+ num2str(tau_cutoff));
                % disp("non_friction_norm: " + num2str(param_out.pi.non_friction_norm));
                % disp("friction_norm: " + num2str(param_out.pi.friction_norm));
                % disp("feasible: " + param_out.pi.feasible);
                % disp("Y_cond: " + num2str(param_out.pi.Y_cond));
                % disp("------");

                disp("start simulating pi_b [" + num2str(q_cutoff) + ", " + num2str(tau_cutoff) + "]");
                tic; 
                try 
                [tau_sim, q_sim] = simulate_parameter(param_out.pi.values.pi_b_act, "pi_b", traj_nr);
                
                duration = toc;
                disp("end simulation - duration = " + num2str(duration)); 

                pi_res.tau = tau_sim;
                pi_res.q = q_sim;
                pi_res.sim_duration = duration;

                res = [];
                
                res.pi = pi_res;
                res.fc_q = q_cutoff;
                res.fc_tau = tau_cutoff;
                res.traj_nr = traj_nr;      
                res.params = param_out;
        
                results = [results {res}];
                save("results/different_simulations_02_27.mat", 'results');
                catch ME
                    disp("error: " + ME.identifier);
                end
            end
        end
    end
end


% for i = [7]
%     tau_meas = measurements{i}.tau;
%     q_meas = measurements{i}.q;
% 
%     tau_meas_1k = tau_meas(1:8:end, :);
%     q_meas_1k = q_meas(1:8:end, :);
% 
%     param_in.measurement.use = true;
%     param_in.measurement.tau = tau_meas_1k;
%     param_in.measurement.q = q_meas_1k;
%     param_in.measurement.qp = [];
%     param_in.measurement.qpp = [];
% 
%     disp("trajectory " + num2str(trajectory_numbers(i)) + ": ");
%     for q_cutoff = 1.9:0.1:2.6
%         for tau_cutoff = q_cutoff:0.1:(q_cutoff + 0.8)
%             param_in.filter.fc_q = q_cutoff;
%             param_in.filter.fc_tau = tau_cutoff;   
% 
%             param_out = identify_parameter(param_in);
% 
%             if param_out.pi.feasible ~= "inertia matrix positive definite (0%) - skew symmetry of Bt - 2C (100%)"
%                 disp("cuttoff: q = " + num2str(q_cutoff) + ", tau = "+ num2str(tau_cutoff));
%                 disp("non_friction_norm: " + num2str(param_out.pi.non_friction_norm));
%                 disp("friction_norm: " + num2str(param_out.pi.friction_norm));
%                 disp("feasible: " + param_out.pi.feasible);
%                 disp("Y_cond: " + num2str(param_out.pi.Y_cond));
%                 disp("------");
%             end
%         end
%     end
% end


% results = [];
% for i = 1:max(size(trajectory_numbers))
% %for traj_nr = [1]
%     % if ~exist('q_meas', 'var') || last_traj_nr ~= traj_nr
%     %     [tau_meas, q_meas] = data_import(traj_nr);
%     % end
%     % last_traj_nr = traj_nr;
% 
%     tau_meas = measurements{i}.tau;
%     q_meas = measurements{i}.q;
% 
%     tau_meas_1k = tau_meas(1:8:end, :);
%     q_meas_1k = q_meas(1:8:end, :);
% 
%     param_in.measurement.use = true;
%     param_in.measurement.tau = tau_meas_1k;
%     param_in.measurement.q = q_meas_1k;
%     param_in.measurement.qp = [];
%     param_in.measurement.qpp = [];
% 
%     param_out = identify_parameter(param_in);
% 
%     disp("trajectory nr. " + num2str(trajectory_numbers(i)));
%     disp("norm: " + num2str(param_out.pi.normal.norm));
%     disp("feasible: " + param_out.pi.normal.feasible);
%     disp("Y_cond: " + num2str(param_out.pi.normal.Y_cond));
%     disp("------");
%     %param_out.pi.normal
% 
%     results = [results {param_out}];
% 
%     %save("evaluation_22_02.mat", 'results');
% end


function opt_ar = get_mu_fmincon_configs()
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
        'Display','off', ...
        'MaxFunctionEvaluations', 50000, ...
        'Algorithm', 'interior-point', ...
        'StepTolerance', 1e-7, ...
        'SpecifyObjectiveGradient',true, ...
        'ObjectiveLimit', 1e+2, ...
        'ConstraintTolerance', 1e-14, ...
        'EnableFeasibilityMode', true);
    opt_ar = [opt_ar {f}];
    
    % f = optimoptions('fmincon', ...
    %     'Display','off', ...
    %     'MaxFunctionEvaluations', 50000, ...
    %     'Algorithm', 'interior-point', ...
    %     'StepTolerance', 1e-7, ...
    %     'SpecifyObjectiveGradient',false, ...
    %     'ObjectiveLimit', 1e+2, ...
    %     'ConstraintTolerance', 1e-14, ...
    %     'EnableFeasibilityMode', true);
    % opt_ar = [opt_ar {f}];

    f = optimoptions('fmincon', ...
        'Display','off', ...
        'MaxFunctionEvaluations', 50000, ...
        'Algorithm', 'sqp', ...
        'StepTolerance', 1e-2, ...
        'SpecifyObjectiveGradient',true, ...
        'ObjectiveLimit', 1e+2, ...
        'ConstraintTolerance', 1e-6, ...
        'EnableFeasibilityMode', true);

    opt_ar = [opt_ar {f}];
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