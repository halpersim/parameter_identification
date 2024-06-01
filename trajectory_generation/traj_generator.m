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
addpath("../simulation/kukalbriiwa_model/matlab/withoutLinAxes/");
addpath("../simulation/kukalbriiwa_model/matlab/withoutLinAxes/parameter_identification/");

param_kuka_lab;

cond_Y_b_bar = zeros(35, 1);
cond_Y_bar = zeros(35, 1);

for i = 1:35
    t = erg{i}.traj;

    Y_bar = Y_bar_from_trajectory(t.a, t.b, 20, @Y, param_robot);
    Y_b_bar = Y_bar_from_trajectory(t.a, t.b, 20, @Y_b, param_robot);

    %disp("cond(Y_b_bar) = " + num2str(cond(Y_b_bar)) + " - cond(Y_bar) = " +  num2str(norm(Y_bar) * norm(pinv(Y_bar))));
    cond_Y_b_bar(i) =  cond(Y_b_bar); 
    cond_Y_bar(i) = norm(Y_bar) * norm(pinv(Y_bar));
end

tab = table(cond_Y_bar, cond_Y_b_bar);


% %for i = 1:max(size(traj))
% for i = 17
% 
%     t = erg{17}.traj;
%     t.T = 20;
%     % t.generate = true;
%     % t.error_func = 1;
%     p = generate_and_test_trajectory(t, false);
% 
%     disp("[" + num2str(i) + "] -  cond = " + num2str(p.traj.cond));
%     disp("Y cond = " + num2str(p.params.pi.Y_b_bar_cond) + ",");
%     disp("pi norm = " + num2str(p.params.pi.non_friction_norm) + ",");
%     disp("feasibility = " + p.params.pi.feasible);
% 
%     % erg = [erg {e}];
%     % disp("... " + int2str(i));
%     % 
%     % save("erg.mat ", 'erg');
% end


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
