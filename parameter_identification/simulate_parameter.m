% this function sets up all paths and sends data to the simulation environment
function [tau_sim, q_sim] = simulate_parameter(param, param_type, traj_nr)
    addpath("../simulation/");
    addpath("../simulation/kukalbriiwa_model/");
    addpath("../parameter_identification/");
    
    evalin('base', 'AUTO_SIM_FLAG = 1;');
    evalin('base', 'G_FACTOR = -1;');

    warning('off', 'MATLAB:rmpath:DirNotFound');
    if param_type == "pi_b"
        evalin('base', 'clear("TEST_MU_PARAMETER");')
        evalin('base', "TEST_PI_B_PARAMETER = 1;");
        evalin('base', "par.pi_b = " + mat2str(param) + ";");

        rmpath("../simulation/kukalbriiwa_model/matlab/withoutLinAxes/");
        addpath("../simulation/kukalbriiwa_model/matlab/withoutLinAxes/dyn_parameter/");
    else if param_type == "mu"
        evalin('base', 'clear("TEST_PI_B_PARAMETER");')
        evalin('base', "TEST_MU_PARAMETER = 1;");
        evalin('base', "par.mu = " + mat2str(param) + ";");

        addpath("../simulation/kukalbriiwa_model/matlab/withoutLinAxes/");
        rmpath("../simulation/kukalbriiwa_model/matlab/withoutLinAxes/dyn_parameter/");
        end
    end

    warning('on','all');
    
    % 1: 17
    % 6: 40
    % 26:30
    % 13:31
    % 17:28
    % 7:35
    % 27:27
    
    traj_durations = zeros(27, 1);
    traj_durations(6) = 40;
    traj_durations(26) = 30;
    traj_durations(13) = 31;
    traj_durations(17) = 28;
    traj_durations(7) = 35;
    traj_durations(27) = 27;

    load("trajectories\all.mat", 'traj');

    T = traj_durations(traj_nr);
    evalin('base', 'TEST_TRAJECTORY = 1;');
    evalin('base', "par.a = " + mat2str(traj{traj_nr}.a) + ";");
    evalin('base', "par.b = " + mat2str(traj{traj_nr}.b) + ";");
    evalin('base', "par.T = " + num2str(T) + ";"); 
    
    if traj_nr == 1
        evalin('base', "TRAJECTORY_TYPE = 3;");
    else
        evalin('base', "TRAJECTORY_TYPE = 4;");
    end

    simulation_output = sim("experiment_simulation", 'ReturnWorkspaceOutputs', 'on');
    
    tau_sim = get(simulation_output.logsout, 'tau').Values.Data;
    q_sim = transpose(squeeze(get(simulation_output.logsout, 'q').Values.Data));
end