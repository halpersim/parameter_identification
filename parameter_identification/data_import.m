function [tau_meas, q_meas] = data_import(traj_nr)
    % traj_nr must be element of [6 26 13 17 7 27] 
    % (only these where actually actuated by the real robot)
    
    %% seconds per trajektorie
    % 1: 17
    % 6: 40
    % 26:30
    % 13:31
    % 17:28
    % 7:35
    % 27:27
    % all trajectories were designed in simulation for T = 20s, except traj. 6 was T = 30s
    % however, in reality these were way too fast and had to be slowed down individually
    % that is why all trajectories have different length
    trajectory_indices = [1 6 26 13 17 7 27];
    seconds_per_trajectory = [17 40 30 31 28 35 27];
    
    %sample rate
    samples_per_sec = 8000;
    %keep a few samples at the beginning to initialize filters
    samples_before = 10000;
    samples_after = 0;

    %% ------ load data
    table = readtable("..\measurements\traj_" + num2str(traj_nr) + ".csv");
    traj_duration = seconds_per_trajectory(find(trajectory_indices == traj_nr, 1));
  
    all = table(:, 2:2:end);
    
    tau = table2array(all(:, 1:7));
    q = table2array(all(:, 8:14));
    %q_p = table2array(all(:, 15:21));
    %q_pp = table2array(all(:, 22:28));


    %each trajectory was carried out 10 or 11 times by hand 
    %therefore, the time between each run is not consistent
    %fortunatelly, there is a distinctive jump in the q data at the start of each run
    %the following lines find this jump and reshape the data accordingly
    %% ---------- reshape data -----------
    q_copy = q;    
    
    find_start_filter_size = 100;
    find_start_filter = zeros(1, find_start_filter_size);
    find_start_filter(1) = 1;
    find_start_filter(find_start_filter_size) = -1;
    
    start_indices = [];
    
    run_duration = traj_duration * samples_per_sec + samples_before + samples_after;
    runs = 0;
    deleted_indices = 0;
    while ~isempty(q_copy) && runs < 20
        filtered = conv(q_copy(:, 2), find_start_filter);
        
        start_idx = find(abs(filtered((find_start_filter_size+1):(end - find_start_filter_size - 1))) > 2.5e-4, 1) - samples_before;
        end_idx = start_idx + run_duration - 1 + samples_before + samples_after;    
    
        if size(q, 1) > end_idx
            runs = runs + 1;
    
            start_indices = [start_indices start_idx+deleted_indices];
    
            deleted_indices = deleted_indices + end_idx;
            q_copy(1:end_idx, :) = [];
        else
            q_copy = [];
        end 
    end
    
    tau_combined = cell2mat(arrayfun( ...
        @(x)(tau(x:(x+run_duration -1), :)), ...
        start_indices, 'UniformOutput', false));
    
    tau_combined = reshape(tau_combined, [run_duration 7 runs]);
    
    q_combined = cell2mat(arrayfun( ...
        @(x)(q(x:(x+run_duration -1), :)), ...
        start_indices, 'UniformOutput', false));
    
    q_combined = reshape(q_combined, [run_duration 7 runs]);

    %% --------- average over all runs (thesis page 29)
    tau_meas = sum(tau_combined, 3);
    tau_meas = tau_meas / runs;
    
    q_meas = sum(q_combined, 3);
    q_meas = q_meas / runs;
end