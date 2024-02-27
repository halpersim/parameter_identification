function traj_out = generate_and_test_trajectory(traj_in, just_masurements, measure)
    % this function generates a new trajectory, tests it in simulation and 
    % then performes the paramter identification on the simulation results
    % alternatively the first or the first two steps can be skipped

    warning('off', 'MATLAB:rmpath:DirNotFound');
    rmpath(genpath("./kukalbriiwa_model/matlab"));
    rmpath(genpath("./parameter_identification"));
    warning('on','all');

    addpath("../simulation/");
    addpath("../simulation/kukalbriiwa_model/");
    addpath("../simulation/kukalbriiwa_model/matlab/withoutLinAxes/");
    addpath("../simulation/kukalbriiwa_model/matlab/withoutLinAxes/parameter_identification/");
    addpath("../parameter_identification/");

    traj_duration = 0;
    if ~just_masurements 

        if isfield(traj_in, 'generate')
    	    %% ------------------- trajectory generation ---------------------
            display = 'off';
            
            min_optimality = 1e+2;
            min_step = 1e-7;
            min_costraint = 1e-14;
            
            display = 'iter';
            min_optimality = 1e+2;
            min_step = 1e-1;
            min_costraint = 1e+1;
            
            fourier_coeffs = 10;
            traj_algorithm = 'interior-point';
            traj_duration = 10;
            
            % a = zeros(7, fourier_coeffs);
            % b = zeros(7, fourier_coeffs);
            % x0 = zeros(1, fourier_coeffs*2*7);
            % traj_cond = 0;
            
            param_kuka_lab;

            fmincon_options = optimoptions('fmincon', ...
                'MaxFunctionEvaluations', 100000, ...
                'Algorithm', traj_algorithm, ...
                'Display', display, ...
                'OptimalityTolerance', min_optimality, ...
                'StepTolerance',  min_step, ...
                'ConstraintTolerance', min_costraint);
        
            [~, a, b, traj_cond] = calculate_trajectory_intern(param_robot, fmincon_options, fourier_coeffs, traj_duration, traj_in.error_func);
        else
            a = traj_in.a;
            b = traj_in.b;
            traj_duration = traj_in.T;
            traj_cond = traj_in.cond;
        end
    
        %% ----------------- Simulation ---------------
        evalin('base', 'AUTO_SIM_FLAG = 1;');
        evalin('base', 'TEST_TRAJECTORY = 1;');
        evalin('base', 'G_FACTOR = 1;');
        evalin('base', "par.a = " + mat2str(a) + ";");
        evalin('base', "par.b = " + mat2str(b) + ";");
        evalin('base', "par.T = " + mat2str(traj_duration) + ";");
    
        simulation_output = sim("experiment_simulation", 'ReturnWorkspaceOutputs', 'on');

        tau = get(simulation_output.logsout, 'tau').Values.Data;
        q = get(simulation_output.logsout, 'q').Values.Data;
        qp = get(simulation_output.logsout, 'q_p').Values.Data;
        qpp = get(simulation_output.logsout, 'q_pp').Values.Data;
    else
        tau = measure.tau;
        q = measure.q;
        qp = measure.qp;
        qpp = measure.qpp; 
        
        a = traj_in.a;
        b = traj_in.b;
        traj_cond = traj_in.cond;
    end



    %% ------------------- Parameter Identification ----------------------
    param_in.mu.calculate = false;
    
    param_in.mu.fmincon_configs = get_mu_fmincon_configs();

    param_in.cut_first_n_elements = 200;
    param_in.filter.use_estimations = false;
    param_in.load_data = false;
    
    param_kuka_lab;
    param_in.param_robot = param_robot;

    param_in.measurement.use = true;
    param_in.measurement.tau = tau;
    param_in.measurement.q = q;
    param_in.measurement.qp = qp;
    param_in.measurement.qpp = qpp;

    param_in.filter.use_filter = true;
    param_in.filter.use_estimations = true;
    param_in.filter.fs = 100;
    param_in.filter.fc_q = 5;
    param_in.filter.fc_tau = 5;
    param_in.filter.butter_order_q = 20;
    param_in.filter.butter_order_tau = 20;

    param_in.with_friction = false;

    param_in.test.trajectory_name = "../parameter_identification/trajectories/strict_53.mat";
    param_in.test.trajectory_duration = 20;
    param_in.test.dyn_param_dir = "../simulation/kukalbriiwa_model/matlab/withoutLinAxes/dyn_parameter/";

    param_out = identify_parameter(param_in);

    %------------------ assign return values

    traj_out.traj.a = a;
    traj_out.traj.b = b;
    traj_out.traj.cond = traj_cond;
    traj_out.traj.T = traj_duration;
    
    traj_out.measurement.tau = param_in.measurement.tau;
    traj_out.measurement.q = param_in.measurement.q;
    traj_out.measurement.qp = param_in.measurement.qp;
    traj_out.measurement.qpp = param_in.measurement.qpp;

    traj_out.params = param_out;
end

function [x0, a, b, traj_cond] = calculate_trajectory_intern(param_robot, fmincon_options, fourier_coeffs, traj_duration, error_func)
    traj_cond = 1e10;

    while traj_cond > 100
        x0 = 4 * (rand(1, fourier_coeffs*2*7) - 0.5);
        [a, b, traj_cond] = calculate_trajectory(param_robot, x0, fmincon_options, fourier_coeffs, traj_duration, error_func);
        
        if traj_cond > 100
            disp("traj failed!");
        end
    end
    disp("traj, done!");
end

function opt_ar = get_mu_fmincon_configs()
    opt_ar = [];

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
    
    f = optimoptions('fmincon', ...
        'Display','off', ...
        'MaxFunctionEvaluations', 50000, ...
        'Algorithm', 'interior-point', ...
        'StepTolerance', 1e-7, ...
        'SpecifyObjectiveGradient',false, ...
        'ObjectiveLimit', 1e+2, ...
        'ConstraintTolerance', 1e-14, ...
        'EnableFeasibilityMode', true);
    opt_ar = [opt_ar {f}];

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
        f = optimoptions('fmincon', ...
        'Display','off', ...
        'MaxFunctionEvaluations', 50000, ...
        'Algorithm', 'sqp', ...
        'StepTolerance', 1e-2, ...
        'SpecifyObjectiveGradient',false, ...
        'ObjectiveLimit', 1e+2, ...
        'ConstraintTolerance', 1e-6, ...
        'EnableFeasibilityMode', true);
    opt_ar = [opt_ar {f}];
end 

function [a,b] = extract_ab(x, coeff_len) 
    a = zeros(7,coeff_len);
    b = zeros(7,coeff_len);

    for i = 1:7
        a(i, :) = x(1+(i-1)*coeff_len : i*coeff_len);
        b(i, :) = x(coeff_len*7 + 1+(i-1)*coeff_len : coeff_len*7 + i*coeff_len);
    end
end