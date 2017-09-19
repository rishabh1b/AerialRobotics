tmax = 14;
num_circles = 1;
omega = 2 * pi * num_circles / (tmax);
amp = 5;
X = [];
Y = [];
X_der = [];
Y_der = [];
for t = 0 : 0.01: tmax
    x_curr = amp * cos(omega * t);
    y_curr_2 = amp * sin(omega * t);
    x_curr_der = - amp * omega * sin(omega * t);
    y_curr_der = amp * omega * cos(omega * t);
    X = [X;x_curr];
    Y = [Y;y_curr_2];
    X_der = [X_der;x_curr_der];
    Y_der = [Y_der;y_curr_der];
end
A1 = [0,0,0,1;...
     (tmax/4)^3, (tmax/4)^2, (tmax/4), 1;...
     0,0,1,0;...
     3 * (tmax/4)^2, 2 * (tmax/4), 1, 0];
 

 A2 = [(tmax/4)^3, (tmax/4)^2, (tmax/4), 1;...
        (tmax/2)^3, (tmax/2)^2, (tmax/2), 1;...
         3 * (tmax/4)^2, 2 * (tmax/4), 1, 0;...
         3 * (tmax/2)^2, 2 * (tmax/2), 1, 0];
     
 A3 = [(tmax/2)^3, (tmax/2)^2, (tmax/2), 1;...
       (3*tmax/4)^3, (3*tmax/4)^2, (3*tmax/4), 1;...
       3 * (tmax/2)^2, 2 * (tmax/2), 1, 0;...
       3 * (3*tmax/4)^2, 2 * (3*tmax/4), 1, 0];

 A4 = [(3*tmax/4)^3, (3*tmax/4)^2, (3*tmax/4), 1;...
        (tmax)^3, (tmax)^2, (tmax), 1;...
        3 * (3*tmax/4)^2, 2 * (3*tmax/4), 1, 0;...
        3 * (tmax)^2, 2 * (tmax), 1, 0;];
    
b_cos_1 = [amp;0;0;-1];
b_cos_2 = [0;-amp;-1;0];
b_cos_3 = [-amp;0;0;1];
b_cos_4 = [0;amp;1;0];

b_sin_1 = [0;amp;0;0];
b_sin_2 = [amp;0;0;-1];
b_sin_3 = [0;-amp;-1;0];
b_sin_4 = [-amp;0;0;0];

C_cos_1 = A1 \ b_cos_1;
C_cos_2 = A2 \ b_cos_2;
C_cos_3 = A3 \ b_cos_3;
C_cos_4 = A4 \ b_cos_4;

C_sin_1 = A1 \ b_sin_1;
C_sin_2 = A2 \ b_sin_2;
C_sin_3 = A3 \ b_sin_3;
C_sin_4 = A4 \ b_sin_4;

t = 0:0.01:tmax;
t_1 = 0:0.01:tmax/4;
t_2 = tmax/4:0.01:tmax/2;
t_3 = tmax/2:0.01:3*tmax/4;
t_4 = 3*tmax/4:0.01:tmax;

figure
subplot(2,2,1)
plot(t,X,'r')
hold on
plot(t_1,polyval(C_cos_1', t_1), 'b')
plot(t_2,polyval(C_cos_2', t_2), 'b')
plot(t_3,polyval(C_cos_3', t_3), 'b')
plot(t_4,polyval(C_cos_4', t_4), 'b')

subplot(2,2,2)
plot(t,X_der,'r');
hold on
plot(t_1,polyval(polyder(C_cos_1'), t_1), 'g')
plot(t_2,polyval(polyder(C_cos_2'), t_2), 'g')
plot(t_3,polyval(polyder(C_cos_3'), t_3), 'g')
plot(t_4,polyval(polyder(C_cos_4'), t_4), 'g')

subplot(2,2,3)
plot(t,Y,'r')
hold on
plot(t_1,polyval(C_sin_1', t_1), 'b')
plot(t_2,polyval(C_sin_2', t_2), 'b')
plot(t_3,polyval(C_sin_3', t_3), 'b')
plot(t_4,polyval(C_sin_4', t_4), 'b')

subplot(2,2,4)
plot(t,Y_der,'r');
hold on
plot(t_1,polyval(polyder(C_sin_1'), t_1), 'g')
plot(t_2,polyval(polyder(C_sin_2'), t_2), 'g')
plot(t_3,polyval(polyder(C_sin_3'), t_3), 'g')
plot(t_4,polyval(polyder(C_sin_4'), t_4), 'g')

figure(2)
plot(t_1,polyval(polyder(polyder(C_sin_1')), t_1), 'g')
plot(t_2,polyval(polyder(polyder(C_sin_2'), t_2)), 'g')
plot(t_3,polyval(polyder(polyder(C_sin_3'), t_3)), 'g')
plot(t_4,polyval(polyder(polyder(C_sin_4'), t_4)), 'g')

% t = 0:0.01:tmax;
% order = 3;
% num_points_interval = 3;
% X_subset_1 = X(1:num_points_interval:numel(t)/2);
% Y_subset_1 = Y(1:num_points_interval:numel(t)/2);
% t_subset_1 = t(1:num_points_interval:numel(t)/2);
% t_subset_1 = t_subset_1';
% % X_subset(1) = 0;
% % X_subset_1(end) = 0;
% % Y_subset_1(1) = 0;
% % Y_subset_1(end) = 0;
% p_v = polyfit(t_subset_1, X_subset_1, order);
% p_v(3) = 0;
% p_v_2 = polyfit(t_subset_1, Y_subset_1, order);
% p_v_2(3) = 0;
% %p_v_2(6) = 0;
% figure
% subplot(2,2,1)
% plot(t,X,'r')
% subplot(2,2,2)
% plot(t,polyval(p_v,t),'r')
% %%
% subplot(2,2,3)
% plot(t,Y,'g')
% subplot(2,2,4)
% plot(t,polyval(p_v_2,t),'g')
