% file that converts the simulation results from the 
% calculated paramters stored in "results/parameter_and_evaluation.ma"
% in all different forms

clear;

%1 7 26 27
traj_nr = 27;
disp_pi = true;
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

csv_name = "traj_27";

%func = @calculate_rms;
%func = "export_csv";
func = "plot_figures";
%func = "export_csv_diff";

load("results/parameter_and_evaluation.mat");

for meas_idx = 1:max(size(meas))
    if meas{meas_idx}.traj_nr == traj_nr
        tau_data = meas{meas_idx}.tau_data;
        tau_filtered = meas{meas_idx}.tau_filtered;
        break;
    end
end

tau_data = [tau_data; ones(1200, 7) .* tau_data(end, :)];

fs = 1000;
fc_tau = 2.7;
butter_order_tau = 20;

[z_tau, p_tau, g_tau] = butter(butter_order_tau, fc_tau/(fs/2));
[tau_sos_filter, g_sos] = zp2sos(z_tau, p_tau, g_tau);
tau_filtered = filtfilt(tau_sos_filter, g_sos, tau_data);


%%-----------------PI TABLE LATEX EXPORT (for thesis)-----------------------
% tab = results{12}.params.pi.values;
% tab(:, 1) = [];
% tab(:, 2) = [];
% tab(:, 1) = round(tab(:,1), 3);
% tab(:, 2) = round(tab(:,2), 2);
% 
% disp("\hline");
% disp("Name & Value & $\sigma$ [\%] & Name & Value ist & $\sigma$ [\%] & Name & Value ist & $\sigma$ [\%]\\");
% disp("\hline");
% for i = 1:19
%     str = "";
%     for k = 0:2
%         na = tab(i + k*19, 3).name;
%         na = na{1};
% 
%         str = str + "$\mathrm{" + na(1:end-1) + "}_" + na(end) + "$ & " + tab(i + k*19, 1).pi_b_ist + " & " + tab(i+k*19, 2).rel_std_dev;
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

%%--------------------- Trajectory q export (for thesis) -------------------------------
% tab = array2table(results{12}.pi.q, "VariableNames", ["q1", "q2", "q3", "q4", "q5", "q6", "q7"]);
% step = transpose(1:2701) * 0.01;
% t2 = table(step);
% tab = [t2 tab];
% all_idx = 1:2701;
% all_idx(1:4:end) = [];
% tab(all_idx, :) = [];
% writetable(tab, "csv/traj_27.csv");
% 

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
      %  k = 1:max(size(tau_filtered));
        
        export = [(k*0.01)' tau_data(k, i) tau_filtered(k, i)];
        leg = ["step", "meas", "filtered"];
        
        if disp_pi 
            for res = 1:max(size(results))
                if results{res}.traj_nr == traj_nr && isfield(results{res}, 'pi') && any(abs(fc_q - results{res}.fc_q) < 0.001)
                    
                    export = [export results{res}.pi.tau(k, i)];
        
                    signal_name = "pi_b";
    
                    leg = [leg signal_name];
                end
            end
        end
    
        mu_lables = ["iter", "sqp", "pi"];
    
        for res = 1:max(size(results))
            for h = 1:max(size(results{res}.mu))
                if results{res}.traj_nr == traj_nr && ismember(h, disp_mu) && ~isempty(results{res}.mu{h}) && any(abs(fc_q - results{res}.fc_q) < 0.001)
                    export = [export results{res}.mu{h}.tau(k, i)];
    
                    signal_name = "mu";
    
                    leg = [leg signal_name];
                end
            end
        end

         all_idx = 1:max(size(export));
        all_idx(1:4:max(size(export))) = [];
        export(all_idx, :) = [];
        tab = array2table(export, "VariableNames", leg);
        writetable(tab, "csv/" + csv_name + "_" + num2str(i) + ".csv");
    end
end

if func == "export_csv_diff"
    for i = 1:7
      %  k = 1:max(size(tau_filtered));
        
        export = [(k*0.01)'];
        leg = ["step"];
        
        if disp_pi 
            for res = 1:max(size(results))
                if results{res}.traj_nr == traj_nr && isfield(results{res}, 'pi') && any(abs(fc_q - results{res}.fc_q) < 0.001)
                    
                    export = [export (results{res}.pi.tau(k, i) - tau_filtered(k,i))];
        
                    signal_name = "pi_b";
    
                    leg = [leg signal_name];
                end
            end
        end
    
        mu_lables = ["iter", "sqp", "pi"];
    
        for res = 1:max(size(results))
            for h = 1:max(size(results{res}.mu))
                if results{res}.traj_nr == traj_nr && ismember(h, disp_mu) && ~isempty(results{res}.mu{h}) && any(abs(fc_q - results{res}.fc_q) < 0.001)
                    export = [export (results{res}.mu{h}.tau(k, i) - tau_filtered(k,i))];
    
                    signal_name = "mu";
    
                    leg = [leg signal_name];
                end
            end
        end
        
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

    for traj_nr = [1 7 26 27]
        load("measurement_data_" + num2str(traj_nr) +  ".mat");


        tau_data(1:1250, :) = [];

        all_idx = 1:max(size(tau_data));
        all_idx(1:10:max(size(tau_data))) = [];
        tau_data(all_idx, :) = [];


        all_idx = 1:max(size(tau_filtered));
        all_idx(1:10:max(size(tau_filtered))) = [];
        tau_filtered(all_idx, :) = [];

        tau_filtered = [zeros(j, 7);  tau_filtered];


        for r = 1:max(size(results)) 
            k = 30:max(size(tau_filtered));

            if results{r}.traj_nr == traj_nr 
                rms_pi = zeros(7,1);
                rms_mu = zeros(7,3);

                for i = 1:7
                    rms_pi(i) = rmse(tau_filtered(k, i), results{r}.pi.tau(k,i));

                    for m = 1:3
                        if ~isempty(results{r}.mu{m}) 
                            rms_mu(i, m) = rmse(tau_filtered(k, i), results{r}.mu{m}.tau(k,i));
                        end
                    end
                end

                results{r}.rms_pi = rms_pi;
                results{r}.rms_mu = rms_mu;

                if sum(rms_pi) < min_rms
                    min_rms = sum(rms_pi);
                    min_r = r;
                    min_type = "pi";
                end

                for m = 1:3
                    if sum(rms_mu(:, m)) > 0.01 && sum(rms_mu(:, m)) < min_rms
                        min_rms = sum(rms_mu(:, m));
                        min_r = r;
                        min_type = "mu " + num2str(m);
                    end
                end
            end
        end
    end
    disp("------------" + num2str(j) + "--------------------");
    for r = 1:max(size(results))
        if results{r}.traj_nr == traj_nr
            disp(num2str(sum(results{r}.rms_pi)) + " - " + mat2str(sum(results{r}.rms_mu)))
        end
    end
end

if func == "plot_figures"
    for i = 1:7
        
        figure('WindowState', window_state);
        hold on;
        title("joint " + num2str(i) + " - trajectory " + num2str(traj_nr));
        %plot(tau_data(k, i));
        %leg = ["raw measurement", "filtered measurement"];

        plot(tau_filtered(k, i), 'LineWidth',2, 'Color', 'g'); 
        leg = ["filtered measurement"];
        
        if disp_pi 
            for res = 1:max(size(results))
                if results{res}.traj_nr == traj_nr && isfield(results{res}, 'pi') && any(abs(fc_q - results{res}.fc_q) < 0.001)
                    
                    plot(results{res}.pi.tau(k, i), 'LineWidth',2);
        
                    signal_name = "\pi_b cutoff(" + num2str(results{res}.fc_q) + "/"+ num2str(results{res}.fc_tau) + ")";
    
                    leg = [leg signal_name];
                end
            end
        end
    
        mu_lables = ["iter", "sqp", "pi"];
    
        for res = 1:max(size(results))
            for h = 1:max(size(results{res}.mu))
                if results{res}.traj_nr == traj_nr && ismember(h, disp_mu) && ~isempty(results{res}.mu{h}) && any(abs(fc_q - results{res}.fc_q) < 0.001)
                    plot(results{res}.mu{h}.tau(k, i), 'LineWidth',2);
    
                    signal_name = "\mu_{" + mu_lables(h) + "} cutoff(" + num2str(results{res}.fc_q) + "/"+ num2str(results{res}.fc_tau) + ")";
    
                    leg = [leg signal_name];
                end
            end
        end
            
        % dere = load("simulation_with_friction_data\non_causal_filter_27_02_2024.mat");
        % dere_tau = dere.data{2}.Values.Data;
        % hund = load("simulation_with_friction_data\real_roll_yaw_27_02_2024.mat");
        % hund_tau = hund.data{2}.Values.Data;
        % 
        % plot(dere_tau(k, i), 'LineWidth', 2);
        % plot(hund_tau(k, i), 'LineWidth', 2);
        % 
        % leg = [leg "\pi_b with noncausal filter" "\pi_b with real roll/yaw"];

        legend(leg);
        hold off;
    end
end