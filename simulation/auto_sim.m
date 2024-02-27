
simulation_output = sim("experiment_simulation.slx", 'ReturnWorkspaceOutputs', 'on');

get(simulation_output.logsout, 'tau').Values.Data
