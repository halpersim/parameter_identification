%% test pi_b paramters for feasibility
% this is done by checking if the inertia matrix is positive definite along a given trajectory
% additionally it is also checked if d(M)/dt - 2*C is skew symmetric along that trajectory
function feasible = test_pi_feasiblity(pi_b, traj_name, traj_duration, param_robot_pitch_roll)
    % Output:
    %   feasible    (string)        - string that states for how many points along the given trajectory
    %                                 the inertia matrix was positive definite and d(M)/dt - 2*C was skew symmetric

    if nargin < 2
        traj_name = "./trajectories/strict_53.mat";
    end
    if nargin < 3
        traj_duration = 20;
    end
    
    [q, qp, qpp] = load_trajectory(traj_name, traj_duration);
    
    %pi_b gets used in this file
    param_kuka_lab;

    param_robot.g = param_robot_pitch_roll.g;
    param_robot.pitch = param_robot_pitch_roll.pitch;
    param_robot.roll = param_robot_pitch_roll.roll;
    param_robot.yaw = param_robot_pitch_roll.yaw;
    
    pos_def = 0;
    skew_sym = 0; 
    for i = 1:size(q,2)
        q_t = q(:, i);
        qp_t = qp(:, i);

        M2 = inertia_matrix(q_t, param_robot);
        Md = inertia_matrix_dt(q_t, qp_t, param_robot);
        C = coriolis_matrix(q_t, qp_t, param_robot);

        skew_symmetry = abs(transpose(qp_t) * (Md - 2*C) * qp_t) < 1e-10;
        inertia_positive_definite = all(eig(M2) > 0);
                
        if inertia_positive_definite
            pos_def = pos_def + 1;
        end
        if skew_symmetry
            skew_sym = skew_sym + 1;
        end
    end 
    
    feasible = "inertia matrix positive definite ("+ num2str(100*pos_def/size(q,2)) + ...
                "%) - skew symmetry of Bt - 2C (" + num2str(100*skew_sym/size(q,2)) + "%)";
end

