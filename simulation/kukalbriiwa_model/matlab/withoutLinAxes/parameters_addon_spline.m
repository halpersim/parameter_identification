%% insert the following code section into your file named "parameters.m"
%% Prepare data for Trajectory generator Spline
tmp=load('trajectory_identification.mat','q','time');
q=tmp.q;
time=tmp.time;

degree=3;
knot=time/time(end);
%Append and prepend the starting and ending knots
param_trajectory_generator_spline=struct();
param_trajectory_generator_spline.degree=degree;
param_trajectory_generator_spline.knot=[zeros(degree-1,1);knot;ones(degree-1,1)];
param_trajectory_generator_spline.position=[repmat(q(1,:),[degree-1,1]);...
  q;...
  repmat(q(end,:),[degree-1,1])];
