%% Intialize 
syms x1 x2 x3 phi theta psi x7 x8 x9 bg1 bg2 bg3 ba1 ba2 ba3 
syms omegam1 omegam2 omegam3 accelm1 accelm2 accelm3 
syms ng1 ng2 ng3 na1 na2 na3 nbg1 nbg2 nbg3 nba1 nba2 nba3
R = [cos(psi)*cos(theta)-sin(phi)*sin(psi)*sin(theta) -cos(phi)*sin(psi) cos(psi)*sin(theta)+cos(theta)*sin(phi)*sin(psi);...
    cos(theta)*sin(psi)+cos(psi)*sin(phi)*sin(theta) cos(phi)*cos(psi) sin(psi)*sin(theta)-cos(psi)*cos(theta)*sin(phi);...
    -cos(phi)*sin(theta) sin(phi) cos(phi)*cos(theta)];
G = [cos(theta) 0 -cos(phi)*sin(theta);0 1 sin(phi);sin(theta) 0 cos(phi)*cos(theta)];
%% Relation between symbols
% x7 = diff(x1);
% x8 = diff(x2);
% x9 = diff(x3);
% nbg1 = diff(bg1);
% nbg2 = diff(bg2);
% nbg3 = diff(bg3);
% nba1 = diff(ba1);
% nba2 = diff(ba2);
% nba3 = diff(ba3);
%% Form the function
vel_vec = [x7;x8;x9];
omegam_vec = [omegam1;omegam2;omegam3];
accelm_vec = [accelm1;accelm2;accelm3];
bias_gyro_vec = [bg1;bg2;bg3];
bias_accel_vec = [ba1;ba2;ba3];
additive_noise_gyro = [ng1;ng2;ng3];
additive_noise_accel = [na1;na2;na3];
bias_gyro_random = [nbg1;nbg2;nbg3];
bias_accel_random = [nba1;nba2;nba3];
gravity = [0;0;9.81];

f = [vel_vec;...
    G \ (omegam_vec - bias_gyro_vec - additive_noise_gyro);...
    gravity + R * (accelm_vec - bias_accel_vec - additive_noise_accel);...
    bias_gyro_random;...
    bias_accel_random];

%% Differentiate w.r.t state
J_s = jacobian(f, [x1 x2 x3 phi theta psi x7 x8 x9 bg1 bg2 bg3 ba1 ba2 ba3]);
%J_s = jacobian(f, [t]);
%% Differentation w.r.t noise
J_n = jacobian(f, [ng1 ng2 ng3 na1 na2 na3 nbg1 nbg2 nbg3 nba1 nba2 nba3]);
%%
J_s = simplify(J_s);
J_n = simplify(J_n);