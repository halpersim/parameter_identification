%% -------- central identification algorithm - (algorithm 2) ---------

function out = identify_parameter(in)
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


    [base_location, ~, ~] = fileparts(mfilename('fullpath'));
    
    %%  -------------- rehape input data ----------------------------
    tau_data = in.measurement.tau;
    q = in.measurement.q;
    qp = in.measurement.qp;
    qpp = in.measurement.qpp;

    q = squeeze(q);
    qp = squeeze(qp);
    qpp = squeeze(qpp);
    
    if size(tau_data, 1) < size(tau_data, 2)
        tau_data = tau_data';
    end
    
    if size(q, 1) < size(q, 2)
        q = q';
        qp = qp';
        qpp = qpp';
    end
    
    
    q_data = q;
    qp_data = qp;
    qpp_data = qpp;


    
    %-------------------  set paths 
    addpath(base_location + "/../simulation/kukalbriiwa_model/matlab/withoutLinAxes/dyn_parameter/");
    addpath(base_location + "/../simulation/kukalbriiwa_model/matlab/withoutLinAxes/parameter_identification/");
    
   
    %-------------- Load Robot Model ----------------------
    param_robot = in.param_robot;
    
          
    %% ++++++++++++++ PI_b ++++++++++++++++++++++
    %--------------- convert measurements ------------------------
    [tau_bar, Y_b_bar, tau_filtered, q_filtered] = convert_measurements(q_data, qp_data, qpp_data, tau_data, param_robot, in.filter, ...
       @(q, q_p, q_pp, param_robot)(Y_b_check_friction(q, q_p, q_pp, param_robot, in.with_friction)), ...
       in.cut_first_n_elements);
    
    %------------------least squares (chapter 2.4.1 / 3.2.3) ------------------
    pi_b_act = lsqlin(Y_b_bar, tau_bar, [], []);
    pi_b_exp = transpose(dynamic_base_parameters_mu_names(param_robot));

    %--------------quality of estimation (chapter 2.4.1) ----------------------
    y_size = size(Y_b_bar);
    unbiased_estimation = (norm(tau_bar - Y_b_bar * pi_b_act)^2) / (y_size(1) - y_size(2));
    abs_std_dev = sqrt(diag(inv(transpose(Y_b_bar) * Y_b_bar)) * unbiased_estimation);
    rel_std_dev = 100 * abs_std_dev ./ abs(pi_b_act);
    
    pi_b_act_raw = pi_b_act;
    if ~in.with_friction
        pi_b_act = add_friction_terms(pi_b_act);
        rel_std_dev = add_friction_terms(rel_std_dev);
    end
    pi_b_diff = pi_b_exp - pi_b_act;

    name = ["ZZR1"; "FV1"; "FS1"; ... 
        "MX2"; "MYR2"; "XXR2"; "XY2"; "XZ2"; "YZ2"; "ZZR2"; "FV2"; "FS2"; ...
        "MX3"; "MYR3"; "XXR3"; "XY3"; "XZ3"; "YZ3"; "ZZR3"; "FV3"; "FS3"; ...
        "MX4"; "MYR4"; "XXR4"; "XY4"; "XZ4"; "YZ4"; "ZZR4"; "FV4"; "FS4"; ...
        "MX5"; "MYR5"; "XXR5"; "XY5"; "XZ5"; "YZ5"; "ZZR5"; "FV5"; "FS5"; ...
        "MX6"; "MYR6"; "XXR6"; "XY6"; "XZ6"; "YZ6"; "ZZR6"; "FV6"; "FS6"; ...
        "MX7"; "MY7"; "XXR7"; "XY7"; "XZ7"; "YZ7"; "ZZ7"; "FV7"; "FS7"];

    diff_without_friction = pi_b_diff;
    diff_without_friction(2:9:end) = [];
    diff_without_friction(2:8:end) = [];

    fric = [pi_b_diff(2:9:end); pi_b_diff(3:9:end)];        

    pi.non_friction_norm = norm(diff_without_friction);
    pi.friction_norm = norm(fric);
    pi.feasible = test_pi_feasiblity(pi_b_act, in.test.trajectory_name, in.test.trajectory_duration, param_robot);
    pi.values = table(pi_b_exp, pi_b_act, pi_b_diff, rel_std_dev, name);
    pi.Y_b_bar_cond = cond(Y_b_bar);
    pi.tau_filtered = tau_filtered;
    pi.q_filtered = q_filtered;

    %% ++++++++++++++++++++ MU ++++++++++++++++++++++++++
    %------------------ Consideration of Physical Feasibility chapter (2.4.2 / 3.2.4) ---------------

    if in.mu.calculate
        [tau_bar, Y_bar] = convert_measurements(q_data, qp_data, qpp_data, tau_data, param_robot, in.filter, @Y, in.cut_first_n_elements);
        mu_exp = get_mu_soll(param_robot);
        mu_vec = [];

        for mu_run = 1:max(size(in.mu.fmincon_configs))
            [mu_act] = calculate_physical_paramers(Y_bar, tau_bar, mu_exp, in.mu.fmincon_configs{mu_run}, false, pi_b_act_raw, param_robot, in.with_friction);

            units = repmat(["kg"; "m";"m";"m"; "kg m²";"kg m²";"kg m²";"kg m²";"kg m²";"kg m²";"FV";"FS"], 7, 1);
            mu_diff = mu_exp - mu_act;

            mu_diff_rel = 100 * mu_diff./mu_exp;

            mu.norm = norm(mu_diff);
            mu.values = table(mu_exp, mu_act, mu_diff, mu_diff_rel, units);

            mu_vec = [mu_vec {mu}];
        end

        %--------- here mu is calculated by trying to reverse the relationship pi_b(mu) --------------------
        % this method was not very successfull 
        % opts = in.mu.fmincon_configs{1};
        % opts.Display = 'off';
        % opts.SpecifyObjectiveGradient = false;
        % 
        % [mu_act] = calculate_physical_paramers(Y_bar, tau_bar, mu_exp, opts, true, pi_b_act, param_robot);
        % 
        % units = repmat(["kg"; "m";"m";"m"; "kg m²";"kg m²";"kg m²";"kg m²";"kg m²";"kg m²";"FS";"FV"], 7, 1);
        % mu_diff = mu_exp - mu_act;
        % mu_diff_rel = 100 * mu_diff./mu_exp;
        % 
        % mu.norm = norm(mu_diff);
        % mu.values = table(mu_exp, mu_act, mu_diff, mu_diff_rel, units);

        % mu_vec = [mu_vec {mu}];
    end

    out.pi = pi;
    if in.mu.calculate
        out.mu = mu_vec;
    end
    
    out.params = in;
end




function Y = Y_b_check_friction(q, q_p, q_pp, param_robot, with_friction)
    Y = Y_b(q, q_p, q_pp, param_robot);

    if ~with_friction
        Y(:, 57) = []; %FS7
        Y(:, 56) = []; %FV7
    
        Y(:, 48) = []; %FS6
        Y(:, 47) = []; %FV6
    
        Y(:, 39) = []; %FS5
        Y(:, 38) = []; %FV5
    
        Y(:, 30) = []; %FS4
        Y(:, 29) = []; %FV4
    
        Y(:, 21) = []; %FS3
        Y(:, 20) = []; %FV3
    
        Y(:, 12) = []; %FS2
        Y(:, 11) = []; %FV2
    
        Y(:, 3) = []; %FS1
        Y(:, 2) = []; %FV1
    end
end

function p = add_friction_terms(pi_vec)
    p = [pi_vec(1); 0; 0; pi_vec(2:8); 0; 0; pi_vec(9:15); 0; 0; pi_vec(16:22); 0; 0; pi_vec(23:29); 0; 0; pi_vec(30:36); 0; 0; pi_vec(37:43); 0; 0];
end

function mu_soll = get_mu_soll(p) 
    mu_soll = zeros(84, 1);

    mu_soll(1) = p.m1;
    mu_soll(2) = p.sp1x;
    mu_soll(3) = p.sp1y;
    mu_soll(4) = p.sp1z;
    mu_soll(5) = p.I1xx;
    mu_soll(6) = p.I1xy;
    mu_soll(7) = p.I1xz;
    mu_soll(8) = p.I1yy;
    mu_soll(9) = p.I1yz;
    mu_soll(10) = p.I1zz;
    mu_soll(11) = 0;
    mu_soll(12) = 0;
    mu_soll(13) = p.m2;
    mu_soll(14) = p.sp2x;
    mu_soll(15) = p.sp2y;
    mu_soll(16) = p.sp2z;
    mu_soll(17) = p.I2xx;
    mu_soll(18) = p.I2xy;
    mu_soll(19) = p.I2xz;
    mu_soll(20) = p.I2yy;
    mu_soll(21) = p.I2yz;
    mu_soll(22) = p.I2zz;
    mu_soll(23) = 0;
    mu_soll(24) = 0;
    mu_soll(25) = p.m3;
    mu_soll(26) = p.sp3x;
    mu_soll(27) = p.sp3y;
    mu_soll(28) = p.sp3z;
    mu_soll(29) = p.I3xx;
    mu_soll(30) = p.I3xy;
    mu_soll(31) = p.I3xz;
    mu_soll(32) = p.I3yy;
    mu_soll(33) = p.I3yz;
    mu_soll(34) = p.I3zz;
    mu_soll(35) = 0;
    mu_soll(36) = 0;
    mu_soll(37) = p.m4;
    mu_soll(38) = p.sp4x;
    mu_soll(39) = p.sp4y;
    mu_soll(40) = p.sp4z;
    mu_soll(41) = p.I4xx;
    mu_soll(42) = p.I4xy;
    mu_soll(43) = p.I4xz;
    mu_soll(44) = p.I4yy;
    mu_soll(45) = p.I4yz;
    mu_soll(46) = p.I4zz;
    mu_soll(47) = 0;
    mu_soll(48) = 0;
    mu_soll(49) = p.m5;
    mu_soll(50) = p.sp5x;
    mu_soll(51) = p.sp5y;
    mu_soll(52) = p.sp5z;
    mu_soll(53) = p.I5xx;
    mu_soll(54) = p.I5xy;
    mu_soll(55) = p.I5xz;
    mu_soll(56) = p.I5yy;
    mu_soll(57) = p.I5yz;
    mu_soll(58) = p.I5zz;
    mu_soll(59) = 0;
    mu_soll(60) = 0;
    mu_soll(61) = p.m6;
    mu_soll(62) = p.sp6x;
    mu_soll(63) = p.sp6y;
    mu_soll(64) = p.sp6z;
    mu_soll(65) = p.I6xx;
    mu_soll(66) = p.I6xy;
    mu_soll(67) = p.I6xz;
    mu_soll(68) = p.I6yy;
    mu_soll(69) = p.I6yz;
    mu_soll(70) = p.I6zz;
    mu_soll(71) = 0;
    mu_soll(72) = 0;
    mu_soll(73) = p.m7;
    mu_soll(74) = p.sp7x;
    mu_soll(75) = p.sp7y;
    mu_soll(76) = p.sp7z;
    mu_soll(77) = p.I7xx;
    mu_soll(78) = p.I7xy;
    mu_soll(79) = p.I7xz;
    mu_soll(80) = p.I7yy;
    mu_soll(81) = p.I7yz;
    mu_soll(82) = p.I7zz;
    mu_soll(83) = 0;
    mu_soll(84) = 0;
end
