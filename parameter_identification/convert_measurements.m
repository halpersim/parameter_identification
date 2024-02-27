function [tau_bar, Y_bar, tau_filtered, q_filtered, qp_est, qpp_est] = convert_measurements(q_data, qp_data, qpp_data, tau_data, param_robot, config, Y_function, cut_first_n_elements)
    if config.use_filter
        %------------------Filter-------------------
        config.cut_first_n_elements = cut_first_n_elements;
        [tau_filtered, q_filtered, qp_est, qpp_est] = filter_data(tau_data, q_data, config);
    else
        q_filtered = q_data;
        tau_filtered = tau_data;
     
        if config.use_estimations 
            %------------------don't use filter,  but, still estimate q_d and q_dd -------------------
            % CENTRAL difference algorithm (thesis p. 29)
            %https://www.mathworks.com/matlabcentral/answers/494553-first-and-second-order-central-difference


            qp_est = zeros(size(q_data));
            qpp_est = zeros(size(q_data));
            
            for i = 2:size(q_data, 1)-1
                qp_est(i, :) = config.fs * 0.5 * (q_filtered(i+1, :) - q_filtered(i-1, :));
                qpp_est(i, :) = config.fs^2 * (q_filtered(i+1, :) - 2 * q_filtered(i, :) + q_filtered(i-1, :));
            end
         else
            qp_est = qp_data;
            qpp_est = qpp_data;
        end

        c = cut_first_n_elements;

        q_filtered = q_filtered(c:end, :);
        qp_est = qp_est(c:end, :);
        qpp_est = qpp_est(c:end, :);
        tau_filtered = tau_filtered(c:end, :);
    end


    %-----------------construct Y_bar, tau_bar - equation (3.47) --------------------
    tau_bar = zeros(7*size(q_filtered, 1), 1);
    Y_bar = zeros(7*size(q_filtered, 1), size(Y_function(zeros(1,7), zeros(1,7), zeros(1,7), param_robot), 2));
      
    for i = 1:size(q_filtered, 1)
        q_i = transpose(q_filtered(i, :));
        qp_i = transpose(qp_est(i, :));
        qpp_i = transpose(qpp_est(i, :));
        tau_i = transpose(tau_filtered(i, :));
    
        tau_bar(((i-1)*7)+1:i*7) = tau_i;
        Y_bar(((i-1)*7)+1:i*7, :) = Y_function(q_i, qp_i, qpp_i, param_robot);
    end
end