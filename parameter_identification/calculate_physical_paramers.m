%%-------------------------------------------------------------
%% Section 3.2.4 (non-linear Optimization Solver) of thesis
%%-------------------------------------------------------------

function [my, grad] = calculate_physical_paramers(Y_bar_obs, tau_bar, my_soll, config, use_pi, pi, params, with_friction)
    fmincon_options = config;
    
    exit_flag = 0;
    
    while exit_flag ~= 1
        [x0, lb, ub] = get_x0_and_bounds();

        % lb = [];
        % ub = [];

        for k = 2:4
            lb(k:12:end) = lb(k:12:end) * 1.5/0.3;
            ub(k:12:end) = ub(k:12:end) * 1.5/0.3;
        end
        x0 = transpose(x0);

        if use_pi 
            err_fc = @(mu)error_function_pi(pi, mu, params);
        else
            err_fc = @(mu)error_function(Y_bar_obs, tau_bar, mu);
        end

        %--------- see equation (3.52)
        [my, ~, exit_flag, options] = fmincon(err_fc, x0, [], [], [], [], lb, ub, @non_linear_boundaries, fmincon_options);    

       exit_flag = 1;
    end
    if ~with_friction
        my = zero_friction_terms(my);
    end

    grad = transpose(calc_grad(Y_bar_obs, tau_bar, my));
end

function [error, grad] = error_function(Y_bar_obs,tau_bar, mu)
    e = tau_bar - Y_bar_obs * dynamic_parameters(mu);
    error = transpose(e) * e;
    grad = calc_grad(Y_bar_obs, tau_bar, mu);
end

function [error, grad] = error_function_pi(pi, my, params) 
    dyn = dynamic_base_parameters_mu_vec(my, params)';
    jac = dynamic_base_parameters_jacobi(my, params);

    e = pi - dyn;
    error = e' * e;
    grad = 2 * e' * jac;
end

function [x0, lb, ub] = get_x0_and_bounds() 
    
    x0 = [6, 0.00, 0.00, 0.00, 0.01, 0.00, 0.00, 0.01, 0.00, 0.001, 0.20, 0.30, ...
        8.0, 0.00, 0.00, 0.00, 0.01, 0.00, 0.00, 0.001, 0.00, 0.01, 0.40, 0.20, ...
        3.0, 0.00, 0.00, 0.00, 0.01, 0.00, 0.00, 0.01, 0.00, 0.001, 0.10, 0.10, ...
        5.0, 0.00, 0.00, 0.00, 0.01, 0.00, 0.00, 0.001, 0.00, 0.01, 0.10, 0.20, ...
        2.0, 0.00, 0.00, 0.00, 0.01, 0.00, 0.00, 0.01, 0.00, 0.001, 0.10, 0.00, ...
        2.0, 0.00, 0.00, 0.00, 0.01, 0.00, 0.00, 0.01, 0.00, 0.01, 0.10, 0.10, ...
        1.0, 0.00, 0.00, 0.00, 0.01, 0.00, 0.00, 0.01, 0.00, 0.01, 0.10, 0.10];
   
    lb = [4.0, -0.1, -0.1, -0.3, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, ...
         5.0, -0.1, -0.3, -0.1, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, ...
         1.0, -0.1, -0.1, -0.3, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, ...
         2.0, -0.1, -0.3, -0.1, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, ...
         0.5, -0.1, -0.1, -0.3, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, ...
         0.5, -0.1, -0.3, -0.1, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, ...
         0.1, -0.1, -0.1, -0.3, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00];

    ub = [ 8, 0.1, 0.1, 0.3, 10, 1, 1, 10, 1, 10, 40, 15, ...
        12.0, 0.1, 0.3, 0.1, 10, 1, 1, 10, 1, 10, 40, 15, ...
         6.0, 0.1, 0.1, 0.3, 10, 1, 1, 10, 1, 10, 40, 15, ...
         8.0, 0.1, 0.3, 0.1, 10, 1, 1, 10, 1, 10, 40, 15, ...
         4.0, 0.1, 0.1, 0.3, 10, 1, 1, 10, 1, 10, 20, 15, ...
         5.0, 0.1, 0.3, 0.1, 10, 1, 1, 10, 1, 10, 20, 15, ...
         3.0, 0.1, 0.1, 0.3, 10, 1, 1, 10, 1, 10, 20, 15];
end

% just for testing in trajectories without friction in simulation
function ret = zero_friction_terms(x)
    ret = x;
    ret(84) = 0; %FS7
    ret(83) = 0; %FV7

    ret(72) = 0; %FS6
    ret(71) = 0; %FV6

    ret(60) = 0; %FS5
    ret(59) = 0; %FV5

    ret(48) = 0; %FS4
    ret(47) = 0; %FV4

    ret(36) = 0; %FS3
    ret(35) = 0; %FV3

    ret(24) = 0; %FS2
    ret(23) = 0; %FV2

    ret(12) = 0; %FS1
    ret(11) = 0; %FV1
end

function [c, ceq] = non_linear_boundaries(x)
    %------------- equations (3.52c - 3.52e)-----------------
    %7 mass values
    %3x7 eigenvalues
    %11x7 gazebo constraints
    
    masses = zeros(1,7);
    eigenvalues = zeros(1,3*7);
    gazebo = zeros(1, 9*7);
    gazebo2 = zeros(1, 2*2*4);

    for i = 0:6
        masses(i+1) = -x(i*12 + 1);
 
        %eigenvalues 
        xx = x(i*12 + 5);
        xy = x(i*12 + 6);
        xz = x(i*12 + 7);
        yy = x(i*12 + 8);
        yz = x(i*12 + 9);
        zz = x(i*12 + 10);

        mat = [[xx xy xz];
            [xy yy yz];
            [xz yz zz]];

        eigenvalues(i*3+1:(i+1)*3) = -eig(mat);

        %gazebo constrains
        gazebo(i*9+1 : i*9+3) = [zz yy xx] - [xx+yy xx+zz yy+zz];
        gazebo(i*9+4) = max([xx yy zz]) - 100 * min([xx yy zz]);
        gazebo(i*9+5) = max(abs([xy xz yz])) - 0.1 * abs(min([xx yy zz]));
        gazebo(i*9+6 : i*9+8) = 0.0001 * [1 1 1] - [zz yy xx];
      %  gazebo(i*9+9) = 0.1 - x(i*12+11);
    end

    i = 1;
    for j = 1:2:5
        jxx = x((j-1)*12+5);
        jyy = x((j-1)*12+8);
        jzz = x((j-1)*12+10);

        gazebo2(i) = 3*jzz - min([jxx jyy]);

        i=i+1;
    end
    for k = 2:2:4
        kxx = x((k-1)*12+5);
        kyy = x((k-1)*12+8);
        kzz = x((k-1)*12+10);

        gazebo2(i) = 3*kyy - min([kxx kzz]);
        i=i+1;
    end

   c = [masses eigenvalues gazebo gazebo2];
   ceq = [];
end

%gradient d(pi(mu))/d(mu)
function grad = calc_grad(Y, tau, mu)
   jacobian = zeros(84, 84);

    for k = 1:7
        q = (k-1)*12;

        pi_d_q = [[1,mu(q+2),mu(q+3),mu(q+4),mu(q+3)^2 + mu(q+4)^2,-mu(q+2)*mu(q+3),-mu(q+2)*mu(q+4),mu(q+2)^2+mu(q+4)^2,-mu(q+3)*mu(q+4),mu(q+2)^2+mu(q+3)^2,0,0];
                [0,mu(q+1),0,0,0,-mu(q+1)*mu(q+3),-mu(q+1)*mu(q+4),2*mu(q+1)*mu(q+2),0,2*mu(q+1)*mu(q+2),0,0];
                [0,0,mu(q+1),0,2*mu(q+1)*mu(q+3),-mu(q+1)*mu(q+2),0,0,-mu(q+1)*mu(q+4),2*mu(q+1)*mu(q+3),0,0];
                [0,0,0,mu(q+1),2*mu(q+1)*mu(q+4),0,-mu(q+1)*mu(q+2),2*mu(q+1)*mu(q+4),-mu(q+1)*mu(q+3),0,0,0];
                [0,0,0,0,1,0,0,0,0,0,0,0];
                [0,0,0,0,0,1,0,0,0,0,0,0];
                [0,0,0,0,0,0,1,0,0,0,0,0];
                [0,0,0,0,0,0,0,1,0,0,0,0];
                [0,0,0,0,0,0,0,0,1,0,0,0];
                [0,0,0,0,0,0,0,0,0,1,0,0];
                [0,0,0,0,0,0,0,0,0,0,1,0];
                [0,0,0,0,0,0,0,0,0,0,0,1]];

        pi_d_q = transpose(pi_d_q);

        jacobian(q+1:q+12,q+1:q+12) = pi_d_q;
    end

    grad = (-2) * transpose(tau - Y * dynamic_parameters(mu)) * Y * jacobian;
end