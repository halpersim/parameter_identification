%% iiwa Parameters from KUKA in Fachvertiefung coordinate system
%--------------------------------------------------------------------------
% This file contains the already identified iiwa parameters from KUKA,
% converted in Fachvertiefung coordinate system
%  author:     Moien.R, TW
%  creation    date:07.05.2020
%---------------------------------
%% World-coordinate system

% [90,0,180] for pascal
% [0,0,0] for tesla
param_robot.yaw = 90 * pi/180;
param_robot.pitch = 0 * pi/180;
param_robot.roll = 180 * pi/180;
param_robot.dw0x = 0;
param_robot.dw0y = 0;
param_robot.dw0z = 0;

% Roboter stands
param_robot.g=9.81;

param_robot.d1          = 0.1575;
param_robot.d2          = 0.2025;
param_robot.d3          = 0.2375;
param_robot.d4          = 0.1825;
param_robot.d5          = 0.2175;
param_robot.d6          = 0.1825;
param_robot.d7          = 0.081;
param_robot.d8          = 0.071;



% %% Length of links (for visualisation)
% param_robot.d1          = 0.1575;
% param_robot.d2          = 0.2025;
% param_robot.d3          = 0.2375;
% param_robot.d4          = 0.1825;
% param_robot.d5          = 0.2175;
% param_robot.d6          = 0.1825;
% param_robot.d7          = 0.081;
param_robot.d8          = 0.071;


%% Dyn Parameter
param_robot.MX1 = 0;
param_robot.MYR1 = 0;
param_robot.MY1 = 0;
param_robot.MZ2 = 0;

param_robot.ZZR1 = pi_b(1);
param_robot.FV1 = pi_b(2);
param_robot.FS1 = pi_b(3);

param_robot.MX2 = pi_b(4);
param_robot.MYR2 = pi_b(5);
param_robot.XXR2 = pi_b(6);
param_robot.XY2 = pi_b(7);
param_robot.XZ2 = pi_b(8);
param_robot.YZ2 = pi_b(9);
param_robot.ZZR2 = pi_b(10);
param_robot.FV2 = pi_b(11);
param_robot.FS2 = pi_b(12);

param_robot.MX3 = pi_b(13);
param_robot.MYR3 = pi_b(14);
param_robot.XXR3 = pi_b(15);
param_robot.XY3 = pi_b(16);
param_robot.XZ3 = pi_b(17);
param_robot.YZR3 = pi_b(18);
param_robot.ZZR3 = pi_b(19);
param_robot.FV3 = pi_b(20);
param_robot.FS3 = pi_b(21);

param_robot.MX4 = pi_b(22);
param_robot.MYR4 = pi_b(23);
param_robot.XXR4 = pi_b(24);
param_robot.XY4 = pi_b(25);
param_robot.XZ4 = pi_b(26);
param_robot.YZ4 = pi_b(27);
param_robot.ZZR4 = pi_b(28);
param_robot.FV4 = pi_b(29);
param_robot.FS4 = pi_b(30);

param_robot.MX5 = pi_b(31);
param_robot.MYR5 = pi_b(32);
param_robot.XXR5 = pi_b(33);
param_robot.XY5 = pi_b(34);
param_robot.XZ5 = pi_b(35);
param_robot.YZR5 = pi_b(36);
param_robot.ZZR5 = pi_b(37);
param_robot.FV5 = pi_b(38);
param_robot.FS5 = pi_b(39);

param_robot.MX6 = pi_b(40);
param_robot.MYR6 = pi_b(41);
param_robot.XXR6 = pi_b(42);
param_robot.XY6 = pi_b(43);
param_robot.XZ6 = pi_b(44);
param_robot.YZ6 = pi_b(45);
param_robot.ZZR6 = pi_b(46);
param_robot.FV6 = pi_b(47);
param_robot.FS6 = pi_b(48);

param_robot.MX7 = pi_b(49);
param_robot.MY7 = pi_b(50);
param_robot.XXR7 = pi_b(51);
param_robot.XY7 = pi_b(52);
param_robot.XZ7 = pi_b(53);
param_robot.YZ7 = pi_b(54);
param_robot.ZZ7 = pi_b(55);
param_robot.FV7 = pi_b(56);
param_robot.FS7 = pi_b(57);

%% Limits
param_robot.tau_max=diag([320.00,320.00,176.00,176.00,110.00,40.00,40.00]);
param_robot.q_limit_upper=[170;120;170;120;170;120;175]*pi/180;
param_robot.q_limit_lower=-param_robot.q_limit_upper;

