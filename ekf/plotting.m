% load('squareOutputs.mat');
% m = matfile('DataSquare.mat');

% load('SlowCircleOutputs.mat');
% m = matfile('DataSlowCircle.mat');

% load('mountainOutputs.mat');
% m = matfile('DataMountain.mat');

load('straightLineOutputs.mat');
m = matfile('DataStraightLine.mat');

%scatter3(positions(2:end,1),positions(2:end,2),positions(2:end,3))
X = (1:335)';
figure(1)
plot(X, Yscy, 'b');
title('StraightLine Y Trajectory')
hold on
plot((1:335)', m.PoseiSAM2(1:335,2), 'r')
legend('EKF', 'iSAM2')

figure(2)
plot(X, Yscx, 'b');
title('StraightLine X Trajectory')
hold on
plot((1:335)', m.PoseiSAM2(1:335,1), 'r')
legend('EKF', 'iSAM2')

figure(3)
plot(X, Yscz, 'b');
title('StraightLine Z Trajectory')
hold on
plot((1:335)', m.PoseiSAM2(1:335,3), 'r')
legend('EKF', 'iSAM2')
%% angles
p = m.PoseiSAM2;
rots = quat2rotm(p(:,4:7));
gteul = [];
[~,~,x] = size(rots);
for i = 1 : x
    gteul = [gteul;getZXYEuler(rots(:,:,i))];
end
%% Phi
figure(4)
plot(X, Yscphi, 'b');
title('StraightLine phi Trajectory')
hold on
plot((1:335)', gteul(1:335,1), 'r')
legend('EKF', 'iSAM2')
%% Theta
figure(5)
plot(X, Ysctheta, 'b');
title('StraightLine theta Trajectory')
hold on
plot((1:335)', gteul(1:335,2), 'r')
legend('EKF', 'iSAM2')
%% Psi
figure(6)
plot(X, Yscpsi, 'b');
title('StraightLine psi Trajectory')
hold on
plot((1:335)', gteul(1:335,3), 'r')
legend('EKF', 'iSAM2')
%% Velocities
figure(7)
plot(X, Yscvelx, 'b');
title('StraightLine Velocity X Trajectory')
hold on
plot((1:335)', m.VeliSAM2(1:335,1), 'r')
legend('EKF', 'iSAM2')

figure(8)
plot(X, Yscvely, 'b');
title('StraightLine Velocity Y Trajectory')
hold on
plot((1:335)', m.VeliSAM2(1:335,2), 'r')
legend('EKF', 'iSAM2')

figure(9)
plot(X, Yscvelz, 'b');
title('StraightLine Velocity Z Trajectory')
hold on
plot((1:335)', m.VeliSAM2(1:335,3), 'r')
legend('EKF', 'iSAM2')
