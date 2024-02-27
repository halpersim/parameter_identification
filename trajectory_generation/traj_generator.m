% this file is the starting point when trying to generate or evaluate new trajectories
% its indented use is that one plays around here and tries out different configurations
% therefore the file is rather chaotic

% currently it loads trajectory 17 and just tests it

% clear
% 
% list = [];
% 
% for i = 1
%     traj.generate = true;
%     traj.error_func = 1;
% 
%     p = generate_and_test_trajectory(traj, false);
% 
%     disp("[" + num2str(i) + "] - " + ...
%         "cond = " + num2str(p.traj.cond) + ", " + ...
%         "Y cond = " + num2str(p.params.pi.normal.Y_cond) + ", " + ...
%         "pi norm = " + num2str(p.params.pi.normal.norm) + ", " + ...
%         "feasibility = " + p.params.pi.normal.feasible);
%     list = [list {p}];
% 
% 
%     save("list.mat ", 'list');
% end

%erg = generate_and_test_trajectory("egal", list{1}.traj)

%e = generate_and_test_trajectory(traj, false);

%e = generate_and_test_trajectory(e.traj, true, e.measurement);

clear;

load("../simulation/trajectories.mat");

%for i = 1:max(size(traj))
for i = 17

    t = erg{17}.traj;
    t.T = 20;
    % t.generate = true;
    % t.error_func = 1;
    p = generate_and_test_trajectory(t, false);
    
    disp("[" + num2str(i) + "] -  cond = " + num2str(p.traj.cond));
    disp("Y cond = " + num2str(p.params.pi.Y_b_bar_cond) + ",");
    disp("pi norm = " + num2str(p.params.pi.non_friction_norm) + ",");
    disp("feasibility = " + p.params.pi.feasible);

    % erg = [erg {e}];
    % disp("... " + int2str(i));
    % 
    % save("erg.mat ", 'erg');
end

% load("erg.mat", 'erg');
% 
% erg2 = erg;
% erg = [];
% for i = 1:2
%     e = generate_and_test_trajectory(erg2{i}.traj, true, erg2{i}.measurement);
% 
%     e.params.pi.normal
%     erg = [erg {e}];
%     % disp("... " + int2str(i));
%     % 
%     % save("erg.mat ", 'erg');
% end
% % 
