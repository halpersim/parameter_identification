%% filter the measured data - filters as in thesis page 29
function [tau_filtered, q_filtered, qp_est, qpp_est] = filter_data(tau_data, q_data, config)
    fs = config.fs;
    fc_q = config.fc_q;
    fc_tau = config.fc_tau;
    butter_order_q = config.butter_order_q;
    butter_order_tau = config.butter_order_tau;
    
    [z_q, p_q, g_q] = butter(butter_order_q, fc_q/(fs/2));
    [z_tau, p_tau, g_tau] = butter(butter_order_tau, fc_tau/(fs/2));
    
    %--------- it is not yet decided which method is the better one ----


    if config.non_causal_filter
        %----------------- non-causal filtering ----------------------------
        [q_sos_filter, q_sos_g] = zp2sos(z_q,p_q,g_q);
        [tau_sos_filter, tau_sos_g] = zp2sos(z_tau, p_tau, g_tau);
    
        q_filtered = filtfilt(q_sos_filter, q_sos_g, q_data);
        tau_filtered = filtfilt(tau_sos_filter, tau_sos_g, tau_data);
    else
        %----------------- causal filtering ----------------------------
        %this is the numerical most stable form of the filter
        %other forms of the filter produced artifacts
        [q_sos_filter] = zp2sos(z_q,p_q,g_q);
        [tau_sos_filter] = zp2sos(z_tau, p_tau, g_tau);
        
        q_filtered = sosfilt(q_sos_filter, q_data, 1);
        tau_filtered = sosfilt(tau_sos_filter, tau_data, 1);
    end

    % estimate qp and qpp from q
    % CENTRAL difference algorithm (thesis p. 29)
    % https://www.mathworks.com/matlabcentral/answers/494553-first-and-second-order-central-difference
    qp_est = zeros(size(q_data));
    qpp_est = zeros(size(q_data));
    
    for i = 2:size(q_data, 1)-1
        qp_est(i, :) = config.fs * 0.5 * (q_filtered(i+1, :) - q_filtered(i-1, :));
        qpp_est(i, :) = config.fs^2 * (q_filtered(i+1, :) - 2 * q_filtered(i, :) + q_filtered(i-1, :));
    end
    
    c = config.cut_first_n_elements;

    q_filtered = q_filtered(c:end, :);
    qp_est = qp_est(c:end, :);
    qpp_est = qpp_est(c:end, :);
    tau_filtered = tau_filtered(c:end, :);
end