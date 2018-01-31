clear all
%%
getJacobian
%%
m = matfile('DataStraightLine.mat');
%m = matfile('DataSlowCircle.mat');
%m = matfile('DataMountain.mat');
%m = matfile('DataSquare.mat');
% m = matfile('DataFastCircle.mat');
%m = matfile('DataMapping.mat');
%% Get the camera and IMU relation
load('CalibParams.mat');
rotCI = quat2rotm(qIMUToC');
transCI = TIMUToC;
HCI = [rotCI transCI;zeros(1,3),1];
%% Get Relevant Info
imu_data = m.IMU;
t_imu = m.IMU(:,11);
t_img = m.TLeftImgs;
%% Sync TimeStamps
% Mesurement(camera) that are in alignment with the IMU readings are kept
% thresh parameter decides the alignment
thresh = 0.001;
n_imu = size(t_imu,1);
n_img = size(t_img,1);
update_ind = [];
chosen_frames_ind = [];
i = 1;
last_i = 0;
for j = 2:n_img
    while i < n_imu
        gap = t_img(j) - t_imu(i);
        if abs(gap) < thresh && i - last_i > 3
           update_ind = [update_ind;i];
           chosen_frames_ind = [chosen_frames_ind;j];
           last_i = i;
           i = i + 1;
           break;
        elseif gap < 0
            % Skip this frame
            i = i + 1;
            break;
        end
        i = i + 1;
    end
end
%% Initialize IMU parameters
% Parameters taken from http://forum.developer.parrot.com/t/what-s-the-detail-of-parameters-in-imu-factory-files/6187/5
gyro_x_bias = 0.012208142813 ;
gyro_y_bias = -0.004909187064; 
gyro_z_bias = 0.00494606804;
acc_x_bias = -0.089717962548; 
acc_y_bias = -0.175323465945;
acc_z_bias = 0.206834236515;
%% Intialize Noise Matrices
Q = 10^-3 * eye(12);
Q(3,3) = 1;
Q(6,6) = 1;
R = 10^-7 * eye(9);
%% Get the measured data
poses = m.PoseiSAM2;
%% Initial state Estimate
p1 = m.PoseiSAM2(2,:);
v1 = m.VeliSAM2(2,:);
pos = p1(1:3);
rot = quat2rotm(p1(4:7));
HWC = [rot pos';zeros(1,3) 1];
HWI = HWC * HCI;
rotIMUFrame = HWI(1:3,1:3);
posIMUFrame = HWI(1:3,4);
eul = getZXYEuler(rotIMUFrame);
xprev = [posIMUFrame(1);posIMUFrame(2);posIMUFrame(3);eul(1);eul(2);eul(3);v1(1);v1(2);v1(3);gyro_x_bias;gyro_y_bias;gyro_z_bias;acc_x_bias;acc_y_bias;acc_z_bias];
covariance = 0.1 * eye(15);
%% Update step related constant matrices
frame_counter = 1;
C = eye(9);
C = [C zeros(9,6)];
%% Main Loop for EKF filtering begins here
positions = zeros(2412,3);
angles = zeros(2412,3);
vels = zeros(2412,3);

positions(1,:) = [0 0 0];
positions(2,:) = [xprev(1) xprev(2) xprev(3)];
%tic
for i = 3:2412
    deltat = t_imu(i) -t_imu(i-1);
    if isempty(imu_data(i,:))
        break;
    end
    gx = imu_data(i,8);
    gy = imu_data(i,9);
    gz = imu_data(i,10);
    ax = imu_data(i,5);
    ay = imu_data(i,6);
    az = imu_data(i,7);
    state_propag = eval(subs(f,[x1,x2,x3,phi,theta,psi,x7,x8,x9,bg1,bg2,bg3,ba1,ba2,ba3,...
                                ng1,ng2,ng3,na1,na2,na3,nbg1,nbg2,nbg3,nba1,nba2,nba3,...
                                omegam1,omegam2,omegam3,accelm1, accelm2, accelm3],...
                                [xprev(1), xprev(2), xprev(3), xprev(4),xprev(5),xprev(6),...
                                 xprev(7), xprev(8), xprev(9), xprev(10),xprev(11),xprev(12),...
                                 xprev(13), xprev(14), xprev(15),...
                                 Q(1,1), Q(2,2), Q(3,3), Q(4,4), Q(5,5), Q(6,6), Q(7,7),...
                                 Q(8,8), Q(9,9), Q(10,10), Q(11,11), Q(12,12),...
                                 gx,gy,gz,ax,ay,az]));
    x_t_bar = xprev + state_propag * deltat;
    lin_state_transition = eye(15) + deltat * eval(subs(J_s,[x1,x2,x3,phi,theta,psi,x7,x8,x9,bg1,bg2,bg3,ba1,ba2,ba3,...
                                ng1,ng2,ng3,na1,na2,na3,nbg1,nbg2,nbg3,nba1,nba2,nba3,...
                                omegam1,omegam2,omegam3,accelm1, accelm2, accelm3],...
                                [xprev(1), xprev(2), xprev(3), xprev(4),xprev(5),xprev(6),...
                                 xprev(7), xprev(8), xprev(9), xprev(10),xprev(11),xprev(12),...
                                 xprev(13), xprev(14), xprev(15),...
                                 Q(1,1), Q(2,2), Q(3,3), Q(4,4), Q(5,5), Q(6,6), Q(7,7),...
                                 Q(8,8), Q(9,9), Q(10,10), Q(11,11), Q(12,12),...
                                 gx,gy,gz,ax,ay,az]));
    lin_noise_transition = deltat * eval(subs(J_n,[x1,x2,x3,phi,theta,psi,x7,x8,x9,bg1,bg2,bg3,ba1,ba2,ba3,...
                                ng1,ng2,ng3,na1,na2,na3,nbg1,nbg2,nbg3,nba1,nba2,nba3,...
                                omegam1,omegam2,omegam3,accelm1, accelm2, accelm3],...
                                [xprev(1), xprev(2), xprev(3), xprev(4),xprev(5),xprev(6),...
                                 xprev(7), xprev(8), xprev(9), xprev(10),xprev(11),xprev(12),...
                                 xprev(13), xprev(14), xprev(15),...
                                 Q(1,1), Q(2,2), Q(3,3), Q(4,4), Q(5,5), Q(6,6), Q(7,7),...
                                 Q(8,8), Q(9,9), Q(10,10), Q(11,11), Q(12,12),...
                                 gx,gy,gz,ax,ay,az]));
    covariance = lin_state_transition * covariance * lin_state_transition' + lin_noise_transition * Q * lin_noise_transition';
    if any(update_ind == i) % Update Step if camera reading available
        frame_idx = chosen_frames_ind(frame_counter);
        frame_counter = frame_counter + 1;
        p1 = m.PoseiSAM2(frame_idx,:);
        if isempty(p1)
            break;
        end
        pos = p1(1:3);
        rot = quat2rotm(p1(4:7));
        if any(isnan(rot))
            break;
        end
        HWC = [rot pos';zeros(1,3),1];
        HWI = HWC * HCI;
        rotIMUFrame = HWI(1:3,1:3);
        posIMUFrame = HWI(1:3,4);
        eul = getZXYEuler(rotIMUFrame);
        vel_meas = m.VeliSAM2(frame_idx,:);
        vel_meas = vel_meas';
        z_t = [posIMUFrame;eul';vel_meas];
        K_t = covariance * C' / (C * covariance * C' + R);
        x_t = x_t_bar + K_t * (z_t - C * x_t_bar);
        covariance = covariance - K_t * C * covariance;
        %xprev = [posIMUFrame(1);posIMUFrame(2);posIMUFrame(3);eul(1);eul(2);eul(3);0;0;0;gyro_x_bias;gyro_y_bias;gyro_z_bias;acc_x_bias;acc_y_bias;acc_z_bias];
    else
        x_t = x_t_bar;
    end
    if any(isnan(x_t))
        break;
    end
    positions(i,:) = x_t(1:3)';
    angles(i,:) = x_t(4:6)';
    vels(i,:) = x_t(7:9)';
    x_prev = x_t;
end
%t = toc
%%
scatter3(positions(2:end,1),positions(2:end,2),positions(2:end,3))
%% Other Plotting Routines
% X = (1:906)';
% Yscy = positions(2:5:end,2);
% figure(1)
% plot(X, positions(2:5:end,2), 'b');
% title('Slow Circle Y Trajectory')
% hold on
% plot((1:900)', m.PoseiSAM2(1:900,2), 'r')
% 
% Yscx = positions(2:5:end,1);
% figure(2)
% plot(X, positions(2:5:end,1), 'b');
% title('Slow Circle X Trajectory')
% hold on
% plot((1:900)', m.PoseiSAM2(1:900,1), 'r')
% 
% Yscz = positions(2:5:end,3);
% figure(3)
% plot(X, positions(2:5:end,3), 'b');
% title('Slow Circle Z Trajectory')
% hold on
% plot((1:900)', m.PoseiSAM2(1:900,3), 'r')
% 
% %% angles
% p = m.PoseiSAM2;
% rots = quat2rotm(p(:,4:7));
% gteul = [];
% [~,~,x] = size(rots);
% for i = 1 : x
%     gteul = [gteul;getZXYEuler(rots(:,:,i))];
% end
% %% Phi
% Yscphi = angles(2:5:end,1);
% figure(4)
% plot(X, angles(2:5:end,1), 'b');
% title('Slow Circle phi Trajectory')
% hold on
% plot((1:900)', gteul(1:900,1), 'r')
% 
% %% Theta
% Ysctheta = angles(2:5:end,2);
% figure(5)
% plot(X, angles(2:5:end,2), 'b');
% title('Slow Circle theta Trajectory')
% hold on
% plot((1:900)', gteul(1:900,2), 'r')
% 
% %% Psi
% Yscpsi = angles(2:5:end,3);
% figure(6)
% plot(X, angles(2:5:end,2), 'b');
% title('Slow Circle psi Trajectory')
% hold on
% plot((1:900)', gteul(1:900,3), 'r')
% 
% %% Velocities
% Yscvelx = vels(2:5:end,1);
% figure(7)
% plot(X, vels(2:5:end,1), 'b');
% title('Slow Circle psi Trajectory')
% hold on
% plot((1:900)', m.VeliSAM2(1:900,1), 'r')
% 
% Yscvely = vels(2:5:end,2);
% figure(8)
% plot(X, vels(2:5:end,2), 'b');
% title('Slow Circle psi Trajectory')
% hold on
% plot((1:900)', m.VeliSAM2(1:900,2), 'r')
% 
% Yscvelz = vels(2:5:end,3);
% figure(9)
% plot(X, vels(2:5:end,3), 'b');
% title('Slow Circle psi Trajectory')
% hold on
% plot((1:900)', m.VeliSAM2(1:900,3), 'r')
