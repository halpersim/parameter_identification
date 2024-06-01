% file that converts the simulation results from the 
% calculated paramters stored in "results/parameter_and_evaluation.ma"
% in all different forms

clear;

%1 7 26 27
traj_nr = 27;
disp_pi = true;
non_causal = 0;
%1 2 3
%different methods how mu is calculated
disp_mu = [1 2 3];

%the results are computed from (q/tau) combinations filtered with different cutoff frequencies
% fc_q may be [1.7, 2, 2.5, 2.6]
% select which cutoff frequencies should be used
fc_q = [2.6];

% range of the data that should be used
k = 1:2700;

window_state = 'maximized';
%window_state = 'normal';

%csv_name = "traj_27";
csv_name = "traj_27_diff";

%func = "calculate_rms";
%func = "export_csv";
%func = "export_csv_diff";
%func = "plot_figures_results";
%func = "plot_figures_res";
func = "nothing";

load("results/final_result_2.mat");
load("results/parameter_and_evaluation.mat");
load("results/different_simulations_02_27.mat");
load("results/final_mu.mat");

for meas_idx = 1:max(size(meas))
    if meas{meas_idx}.traj_nr == traj_nr
        tau_data = meas{meas_idx}.tau_data;
        tau_filtered = meas{meas_idx}.tau_filtered;
        break;
    end
end

tau_data = [tau_data; ones(1200, 7) .* tau_data(end, :)];

fs = 1000;
fc_tau = 3.5;
butter_order_tau = 20;

[z_tau, p_tau, g_tau] = butter(butter_order_tau, fc_tau/(fs/2));
[tau_sos_filter, g_sos] = zp2sos(z_tau, p_tau, g_tau);
tau_filtered = filtfilt(tau_sos_filter, g_sos, tau_data);


%%-----------------MIN - MAX Measured of each joint
minmax = (max(tau_filtered) - min(tau_filtered)) * 0.1;
roman = ["i", "ii", "iii", "iv", "v", "vi", "vii"];
for i = 1:7
    disp("\def\varj" + roman(i) + "{" + minmax(i) + "}");
end


%%-----------------PI TABLE LATEX EXPORT (for thesis)-----------------------
% tab = res.params.pi.values;
% tab(:, 1) = [];
% tab(:, 2) = [];
% tab(:, 1) = round(tab(:,1), 3);
% tab(:, 2) = round(tab(:,2), 2);
% 
% disp("\hline");
% disp("Name & Value & $\sigma_{\hat{x}_j, r}$ [\%] & Name & Value & $\sigma_{\hat{x}_j, r}$ [\%] & Name & Value & $\sigma_{\hat{x}_j, r}$ [\%]\\");
% disp("\hline");
% for i = 1:19
%     str = "";
%     for k = 0:2
%         na = tab(i + k*19, 3).name;
%         na = na{1};
% 
%         str = str + "$\mathrm{" + na(1:end-1) + "}_" + na(end) + "$ & " + tab(i + k*19, 1).pi_b_act + " & " + tab(i+k*19, 2).rel_std_dev;
% 
%         if k == 2
%             str = str + "\\";
%         else
%             str = str + " & ";
%         end
%     end
% 
%     disp(str);
% end
% disp("\hline");

%%-----------------MU TABLE LATEX EXPORT (for thesis)-----------------------
% tab = mu_l.values;
% tab(:, 1) = [];
% tab(:, 2) = [];
% tab(:, 1) = round(tab(:,1), 3);
% tab(:, 2) = round(tab(:,2), 2);
% 
% names = ["m_i", "r^{c_i}_{i, x}", "r^{c_i}_{i, y}", "r^{c_i}_{i, z}", "I^i_{i, xx}", "I^i_{i,xy}", "I^i_{i,xz}", "I^i_{i,yy}", "I^i_{i,yz}", "I^i_{i,zz}", "F_{iv}", "F_{is}"];
% 
% disp("\hline");
% disp("Par. & Link 1 & Link 2 & Link 3 & Link 4 & Link 5 & Link 6 & Link 7 \\");
% disp("\hline");
% for i = 1:12
%     str = "$" +  names(i) + "$ & ";
%     for k = 0:6
%         value = tab(i + k*12, 1).mu_act;
% 
%         str = str + "$" +  num2str(value) + "$";
%         if k == 6
%             str = str + "\\";
%         else
%             str = str + " & ";
%         end
%     end
% 
%     disp(str);
% end
% disp("\hline");

%%--------------------- test Trajectory q export (for thesis) -------------------------------
% load("trajectories\strict_53.mat");
% w = 2 * pi / 20;
% q = zeros(201, 7);
% a = best_a;
% b = best_b;
% 
% step = 0:0.1:20;
% l = (1:5)';
% 
% q = (a * (sin(w * l * step) ./ (w*l)) - b * (cos(w*l*step) ./(w*l)))';
% 
% tab = array2table(q, "VariableNames", ["q1", "q2", "q3", "q4", "q5", "q6", "q7"]);
% step = step';
% t2 = table(step);
% tab = [t2 tab];
% writetable(tab, "csv/traj_test.csv");
% for type = ["a", "b"]
%     disp("\hline");
% 
%     str = "";
%     for k = 0:7
%         if k > 0
%             str = str + "$q_{" + num2str(k) + "}$";
%         end
%         if k == 7
%             str = str + "\\"; 
%         else
%             str = str + " & ";
%         end
%     end
% 
%     disp(str);
%     disp("\hline");
%     for i = 1:numel(l)
%         str = "";
%         for k = 0:7
%             if k > 0
%                 if type == "a" 
%                     value = best_a(k, i);
%                 else
%                     value = best_b(k, i);
%                 end
% 
%                 str = str + "$" + num2str(round(value, 5)) + "$";
%             else
%                 str = str + "$\mathrm{" + type + "}_{"+ num2str(i) + "}$";
%             end
%             if k == 7
%                 str = str + "\\"; 
%             else
%                 str = str + " & ";
%             end
%         end
%         disp(str);
%     end
%     if type == "a"
%         disp("\hline");
%         disp("\\");
%     end
% end

%%--------------------- Trajectory q export (for thesis) -------------------------------
% tab = array2table(results{12}.pi.q, "VariableNames", ["q1", "q2", "q3", "q4", "q5", "q6", "q7"]);
% step = transpose(1:2701) * 0.01;
% t2 = table(step);
% tab = [t2 tab];
% all_idx = 1:2701;
% all_idx(1:4:end) = [];
% tab(all_idx, :) = [];
% writetable(tab, "csv/traj_27.csv");

%the actual trajectory starts at measurent 1263
%before that, the robot just stands still -> this data is used to
%initialize the filters
tau_data(1:1263, :) = [];
tau_filtered(1:1263, :) = [];

all_idx = 1:max(size(tau_data));
all_idx(1:10:max(size(tau_data))) = [];
tau_data(all_idx, :) = [];

all_idx = 1:max(size(tau_filtered));
all_idx(1:10:max(size(tau_filtered))) = [];
tau_filtered(all_idx, :) = [];

%tau_filtered(end-40:end, :) = [];

if func == "export_csv"
    for i = 1:7        
        export = [(k*0.01)' tau_data(k, i) tau_filtered(k, i) res.pi.tau(k, i) res.mu.tau(k, i)];
        leg = ["step", "meas", "filtered", "pi_b", "mu"];

        all_idx = 1:max(size(export));
        all_idx(1:4:max(size(export))) = [];
        export(all_idx, :) = [];
        tab = array2table(export, "VariableNames", leg);
        writetable(tab, "csv/" + csv_name + "_" + num2str(i) + ".csv");
    end
end

if func == "export_csv_diff"
    for i = 1:7
        export = [(k*0.01)' (res.pi.tau(k, i)-tau_filtered(k, i)) (res.mu.tau(k, i)-tau_filtered(k, i))];
        leg = ["step", "pi_b", "mu"];
              
        all_idx = 1:max(size(export));
        all_idx(1:4:max(size(export))) = [];
        export(all_idx, :) = [];
        tab = array2table(export, "VariableNames", leg);
        writetable(tab, "csv/" + csv_name + "_" + num2str(i) + ".csv");
    end
end

if func == "calculate_rms"
    min_rms = 1e10;
    min_r = 0;
    min_type = "";

    for traj_nr = [27]
        % load("measurement_data_" + num2str(traj_nr) +  ".mat");
        % 
        % 
        % tau_data(1:1250, :) = [];
        % 
        % all_idx = 1:max(size(tau_data));
        % all_idx(1:10:max(size(tau_data))) = [];
        % tau_data(all_idx, :) = [];
        % 
        % 
        % all_idx = 1:max(size(tau_filtered));
        % all_idx(1:10:max(size(tau_filtered))) = [];
        % tau_filtered(all_idx, :) = [];
        % 
        % tau_filtered = [zeros(j, 7);  tau_filtered];


        for r = 1:max(size(results)) 
            %k = 30:max(size(tau_filtered));

            if results{r}.traj_nr == traj_nr 
                rms_pi = zeros(7,1);
                % rms_mu = zeros(7,3);

                for i = 1:7
                    rms_pi(i) = rmse(tau_filtered(k, i), results{r}.pi.tau(k,i));

                    % for m = 1:3
                    %     if ~isempty(results{r}.mu{m}) 
                    %         rms_mu(i, m) = rmse(tau_filtered(k, i), results{r}.mu{m}.tau(k,i));
                    %     end
                    % end
                end

                results{r}.rms_pi = rms_pi;
%                results{r}.rms_mu = rms_mu;

                if sum(rms_pi) < min_rms
                    min_rms = sum(rms_pi);
                    min_r = r;
                    min_type = "pi";
                end

                % for m = 1:3
                %     if sum(rms_mu(:, m)) > 0.01 && sum(rms_mu(:, m)) < min_rms
                %         min_rms = sum(rms_mu(:, m));
                %         min_r = r;
                %         min_type = "mu " + num2str(m);
                %     end
                % end
            end
        end
    end
    disp("------------" + num2str(j) + "--------------------");
    for r = 1:max(size(results))
        if results{r}.traj_nr == traj_nr
            disp(num2str(sum(results{r}.rms_pi)));% + " - " + mat2str(sum(results{r}.rms_mu)))
        end
    end
end

if func == "plot_figures_results"
    for i = 1:7
        
        figure('WindowState', window_state);
        hold on;
        title("joint " + num2str(i) + " - trajectory " + num2str(traj_nr));
        %plot(tau_data(k, i));
        %leg = ["raw measurement", "filtered measurement"];

        plot(tau_filtered(k, i), 'LineWidth',1, 'Color', 'g'); 
        leg = ["filtered measurement"];
        
        
        %for res = 1:max(size(results))
        for res = 142
            %  if results{res}.params.params.filter.non_causal_filter == non_causal
                if results{res}.params.params.filter.non_causal_filter == 1
                    color = 'r';
                else
                    color = 'b';
                end

                plot(results{res}.pi.tau(k, i), 'LineWidth',1, 'Color', color);
    
                signal_name = "\pi_b cutoff(" + num2str(results{res}.fc_q) + "/"+ num2str(results{res}.fc_tau) + ")";

               % leg = [leg signal_name];
           % end
       end
        
         
        % dere = load("simulation_with_friction_data\non_causal_filter_27_02_2024.mat");
        % dere_tau = dere.data{2}.Values.Data;
        % hund = load("simulation_with_friction_data\real_roll_yaw_27_02_2024.mat");
        % hund_tau = hund.data{2}.Values.Data;
        % 
        % plot(dere_tau(k, i), 'LineWidth', 2);
        % plot(hund_tau(k, i), 'LineWidth', 2);

     %   leg = [leg "\pi_b with noncausal filter" "\pi_b with real roll/yaw"];

       % legend(leg);
        hold off;
    end
end

if func == "plot_figures_res"
    pi_rms = zeros(7,1);
    mu_rms = zeros(7,1);

    for i = 1:7
        
        figure('WindowState', window_state);
        hold on;
        title("joint " + num2str(i) + " - trajectory " + num2str(traj_nr));
        %plot(tau_data(k, i));
        %leg = ["raw measurement", "filtered measurement"];

        plot(tau_filtered(k, i), 'LineWidth',1, 'Color', 'g'); 
        leg = ["filtered measurement"];
               
        plot(res.pi.tau(k, i), 'LineWidth',1);

        signal_name = "\pi_b cutoff(" + num2str(res.fc_q) + "/"+ num2str(res.fc_tau) + ")";
        leg = [leg signal_name];

        pi_rms(i) = rmse(tau_filtered(k, i), res.pi.tau(k,i));
        mu_rms(i) = rmse(tau_filtered(k, i), res.mu.tau(k,i));
        plot(res.mu.tau(k, i), 'LineWidth', 1);

        signal_name = "\mu sqp";
        leg = [leg signal_name];
        % for m = 1:max(size(res.mu))
        %     plot(res.mu{m}.tau(k,i), 'LineWidth',1);
        % 
        %     signal_name = "\mu [" + m + "]";    
        %     leg = [leg signal_name];
        % end
        % 
        legend(leg);
        hold off;
    end
end