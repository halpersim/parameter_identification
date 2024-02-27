%% ------------ generate good trajectory - chapter (3.3) --------------
function [a,b,cond] = calculate_trajectory(param_robot, x0, fmincon_options, coeff_len, duration, error_func)
   w = 2 * pi/duration;
   
    if nargin < 6
        error_func = 1;
    end

   %------------ optimization problem (3.55) in the thesis --------------------------
   [x_end, cond] = fmincon(@(x)error_function(x, w, coeff_len, duration, param_robot, error_func), x0, [], [], [], [], [], [], @(x)non_linear_boundaries(w, coeff_len, param_robot, x), fmincon_options);

   [a,b] = extract_ab(x_end, coeff_len);
end

function error = error_function(x, w, coeff_len, duration, param_robot, error_func)
    [a, b] = extract_ab(x, coeff_len);

    s = size(trajectory_Y_b_reduced(zeros(7,coeff_len), zeros(7,coeff_len), w, 0, param_robot));

    Y_bar = zeros(s(1)*1000, s(2));

    for i = 1:100*duration
        t = (i-1)*0.01;

        Y_bar(((i-1)*7)+1 : i*7, :) = trajectory_Y_b_reduced(a, b, w, t, param_robot);
    end

    %------ different error functions ---------------
    % according to [6] chapter 12.3.4.1.
    %
    % [6] W. Khalil and E. Dombre, Modeling, Identification & Control of Robots. Kogan Page
    %     Science, 2002, Chapters 9, 12 and Appendix 4.
    %
    % these aren't mentioned in the thesis, because
    % different error functions didn't result in better results (at least according to simulations)
    switch error_func
        case 1
            error = cond(Y_bar);

        case 2 
            error = cond(Y_bar); 

            Y_bar = abs(Y_bar);
            error = error + 1e-6 * max(max(Y_bar)) / min(Y_bar(Y_bar > 1e-6));

        case 3
            error = cond(Y_bar * get_order_of_magnidute_estimates());
               
        case 4
            error = cond(Y_bar) + 0.1 / sqrt(min(abs(eig(transpose(Y_bar) * Y_bar))));

        otherwise
            error = cond(Y_bar);
    end

end

function [c, ceq] = non_linear_boundaries(w, coeff_len, param_robot, x)
    [a, b] = extract_ab(x, coeff_len);

    %---------------- (3.55b) - (3.55f) -----------------------------

    %according to https://www.kuka.com/-/media/kuka-downloads/imported/8350ff3ca11642998dbdc81dcc2ed44c/0000246833_de.pdf?rev=d7c0f2f14a184ff8
    QP_MAX = deg2rad([85 85 100 75 130 135 135]);

    ceq_al = zeros(1,7);
    ceq_b = zeros(1,7);
    ceq_la = zeros(1,7);

    for i = 1:7
        for l = 1:coeff_len
            ceq_al(i) = ceq_al(i) + b(i, l) / (l * w);
            ceq_b(i) = ceq_b(i) + a(i, l);
            ceq_la(i) = ceq_la(i) + b(i, l) * (l * w);
        end
    end

    ceq = [ceq_al ceq_b ceq_la];

    c_q_max = zeros(1,7);
    c_qp_max = zeros(1,7);
    c_indivitual_a = zeros(1, 7);
    c_indivutual_b = zeros(1, 7);

    for i = 1:7
        for l = 1:coeff_len
            c_q_max(i) = c_q_max(i) + sqrt(a(i,l)^2 + b(i,l)^2)/l;
            c_qp_max(i) = c_qp_max(i) + sqrt(a(i,l)^2 + b(i,l)^2);

            c_indivitual_a(i) = abs(a(i,l)) - min(l * w / coeff_len * param_robot.q_limit_upper(i), QP_MAX(i));
            c_indivutual_b(i) = abs(b(i,l)) - min(l * w / coeff_len * param_robot.q_limit_upper(i), QP_MAX(i));
        end
    
        c_q_max(i) = c_q_max(i) - w * param_robot.q_limit_upper(i);
        c_qp_max(i) = c_qp_max(i) - QP_MAX(i);
    end

    c = [c_q_max c_qp_max c_indivitual_a c_indivutual_b];
end


function [a,b] = extract_ab(x, coeff_len) 
    a = zeros(7,coeff_len);
    b = zeros(7,coeff_len);

    for i = 1:7
        a(i, :) = x(1+(i-1)*coeff_len : i*coeff_len);
        b(i, :) = x(coeff_len*7 + 1+(i-1)*coeff_len : coeff_len*7 + i*coeff_len);
    end
end

function Z = get_order_of_magnidute_estimates() 
    Z = diag([    ...
        1.0000e-01
   1.0000e-04
   1.0000e+01
   1.0000e+00
   1.0000e-05
   1.0000e-06
   1.0000e-03
   1.0000e+00
   1.0000e-03
   1.0000e-02
   1.0000e-02
   1.0000e-05
   1.0000e-04
   1.0000e-02
   1.0000e-01
   1.0000e-03
   1.0000e+00
   1.0000e+00
   1.0000e-04
   1.0000e-05
   1.0000e-02
   1.0000e+00
   1.0000e-04
   1.0000e-01
   1.0000e-02
   1.0000e-06
   1.0000e-05
   1.0000e-02
   1.0000e-02
   1.0000e-03
   1.0000e-01
   1.0000e-02
   1.0000e-05
   1.0000e-05
   1.0000e-03
   1.0000e-02
   1.0000e-03
   1.0000e-04
   1.0000e-05
   1.0000e-07
   1.0000e-05
   1.0000e-06
   1.0000e-03
        ]);
        
end
