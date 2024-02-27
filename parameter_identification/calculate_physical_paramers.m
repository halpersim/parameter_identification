%%-------------------------------------------------------------
%% Section 3.2.4 (non-linear Optimization Solver) of thesis
%%-------------------------------------------------------------

function [my, grad] = calculate_physical_paramers(Y_bar_obs, tau_bar, my_soll, config, use_pi, pi, params, with_friction)
    fmincon_options = config;
    
    exit_flag = 0;
    
    while exit_flag ~= 1
        [x0, lb, ub] = get_x0_and_bounds();

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
   
    lb = [4.0, -0.3, -0.3, -0.3, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, ...
         5.0, -0.3, -0.3, -0.3, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, ...
         1.0, -0.3, -0.3, -0.3, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, ...
         2.0, -0.3, -0.3, -0.3, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, ...
         0.5, -0.3, -0.3, -0.3, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, ...
         0.5, -0.3, -0.3, -0.3, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, ...
         0.1, -0.3, -0.3, -0.3, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00];

    ub = [8, 0.3, 0.3, 0.3, 1, 1, 1, 1, 1, 1, 40, 15, ...
        12.0, 0.3, 0.3, 0.3, 1, 1, 1, 1, 1, 1, 40, 15, ...
        6.0, 0.3, 0.3, 0.3, 1, 1, 1, 1, 1, 1, 40, 15, ...
        8.0, 0.3, 0.3, 0.3, 1, 1, 1, 1, 1, 1, 40, 15, ...
        4.0, 0.3, 0.3, 0.3, 1, 1, 1, 1, 1, 1, 20, 15, ...
        5.0, 0.3, 0.3, 0.3, 1, 1, 1, 1, 1, 1, 20, 15, ...
        3.0, 0.3, 0.3, 0.3, 1, 1, 1, 1, 1, 1, 20, 15];
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


%conversion pi(mu) 
function ret = get_dynamic_paramters(m)
    ret(1)=m(1);
    ret(2)=m(1) * m(2);
    ret(3)=m(1) * m(3);
    ret(4)=m(1) * m(4);
    ret(5)=m(5) + m(1)*(m(3)^2 + m(4)^2);
    ret(6)=m(6) - m(1) * m(2) * m(3);
    ret(7)=m(7) - m(1) * m(2) * m(4);
    ret(8)=m(8) + m(1)*(m(2)^2 + m(4)^2);
    ret(9)=m(9) - m(1) * m(3) * m(4);
    ret(10)=m(10) + m(1)*(m(2)^2 + m(3)^2);
    ret(11)=sym(0);
    ret(12)=sym(0);
    ret(13)=m(13);
    ret(14)=m(13) * m(14);
    ret(15)=m(13) * m(15);
    ret(16)=m(13) * m(16);
    ret(17)=m(17) + m(13)*(m(15)^2 + m(16)^2);
    ret(18)=m(18) - m(13) * m(14) * m(15);
    ret(19)=m(19) - m(13) * m(14) * m(16);
    ret(20)=m(20) + m(13)*(m(14)^2 + m(16)^2);
    ret(21)=m(21) - m(13) * m(15) * m(16);
    ret(22)=m(22) + m(13)*(m(14)^2 + m(15)^2);
    ret(23)=sym(0);
    ret(24)=sym(0);
    ret(25)=m(25);
    ret(26)=m(25) * m(26);
    ret(27)=m(25) * m(27);
    ret(28)=m(25) * m(28);
    ret(29)=m(29) + m(25)*(m(27)^2 + m(28)^2);
    ret(30)=m(30) - m(25) * m(26) * m(27);
    ret(31)=m(31) - m(25) * m(26) * m(28);
    ret(32)=m(32) + m(25)*(m(26)^2 + m(28)^2);
    ret(33)=m(33) - m(25) * m(27) * m(28);
    ret(34)=m(34) + m(25)*(m(26)^2 + m(27)^2);
    ret(35)=sym(0);
    ret(36)=sym(0);
    ret(37)=m(37);
    ret(38)=m(37) * m(38);
    ret(39)=m(37) * m(39);
    ret(40)=m(37) * m(40);
    ret(41)=m(41) + m(37)*(m(39)^2 + m(40)^2);
    ret(42)=m(42) - m(37) * m(38) * m(39);
    ret(43)=m(43) - m(37) * m(38) * m(40);
    ret(44)=m(44) + m(37)*(m(38)^2 + m(40)^2);
    ret(45)=m(45) - m(37) * m(39) * m(40);
    ret(46)=m(46) + m(37)*(m(38)^2 + m(39)^2);
    ret(47)=sym(0);
    ret(48)=sym(0);
    ret(49)=m(49);
    ret(50)=m(49) * m(50);
    ret(51)=m(49) * m(51);
    ret(52)=m(49) * m(52);
    ret(53)=m(53) + m(49)*(m(51)^2 + m(52)^2);
    ret(54)=m(54) - m(49) * m(50) * m(51);
    ret(55)=m(55) - m(49) * m(50) * m(52);
    ret(56)=m(56) + m(49)*(m(50)^2 + m(52)^2);
    ret(57)=m(57) - m(49) * m(51) * m(52);
    ret(58)=m(58) + m(49)*(m(50)^2 + m(51)^2);
    ret(59)=sym(0);
    ret(60)=sym(0);
    ret(61)=m(61);
    ret(62)=m(61) * m(62);
    ret(63)=m(61) * m(63);
    ret(64)=m(61) * m(64);
    ret(65)=m(65) + m(61)*(m(63)^2 + m(64)^2);
    ret(66)=m(66) - m(61) * m(62) * m(63);
    ret(67)=m(67) - m(61) * m(62) * m(64);
    ret(68)=m(68) + m(61)*(m(62)^2 + m(64)^2);
    ret(69)=m(69) - m(61) * m(63) * m(64);
    ret(70)=m(70) + m(61)*(m(62)^2 + m(63)^2);
    ret(71)=sym(0);
    ret(72)=sym(0);
    ret(73)=m(73);
    ret(74)=m(73) * m(74);
    ret(75)=m(73) * m(75);
    ret(76)=m(73) * m(76);
    ret(77)=m(77) + m(73)*(m(75)^2 + m(76)^2);
    ret(78)=m(78) - m(73) * m(74) * m(75);
    ret(79)=m(79) - m(73) * m(74) * m(76);
    ret(80)=m(80) + m(73)*(m(74)^2 + m(76)^2);
    ret(81)=m(81) - m(73) * m(75) * m(76);
    ret(82)=m(82) + m(73)*(m(74)^2 + m(75)^2);
    ret(83)=sym(0);
    ret(84)=sym(0);
end

%gradient d(pi(mu))/d(mu)
function grad = calc_grad(Y, tau, my)
   jacobian = zeros(84, 84);

    for k = 1:7
        q = (k-1)*12;

        pi_d_q = [[1,my(q+2),my(q+3),my(q+4),my(q+3)^2 + my(q+4)^2,-my(q+2)*my(q+3),-my(q+2)*my(q+4),my(q+2)^2+my(q+4)^2,-my(q+3)*my(q+4),my(q+2)^2+my(q+3)^2,0,0];
                [0,my(q+1),0,0,0,-my(q+1)*my(q+3),-my(q+1)*my(q+4),2*my(q+1)*my(q+2),0,2*my(q+1)*my(q+2),0,0];
                [0,0,my(q+1),0,2*my(q+1)*my(q+3),-my(q+1)*my(q+2),0,0,-my(q+1)*my(q+4),2*my(q+1)*my(q+3),0,0];
                [0,0,0,my(q+1),2*my(q+1)*my(q+4),0,-my(q+1)*my(q+2),2*my(q+1)*my(q+4),-my(q+1)*my(q+3),0,0,0];
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

    grad = 2 * (transpose(dynamic_parameters(my)) * transpose(Y) - transpose(tau)) * Y * jacobian;
end