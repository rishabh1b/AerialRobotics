tmax = 12;
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

f = @(t) amp * omega * cos(omega * t);
fdash = @(t) -amp * omega^2 * sin(omega * t);

buffer = 1;
t_init = buffer;
t_last = tmax - buffer;

A1 = [0,0,0,1;...
     (t_init)^3, (t_init)^2, (t_init), 1;...
     0,0,1,0;...
     3 * (t_init)^2, 2 * (t_init), 1, 0];
 
A4 = [(t_last)^3, (t_last)^2, (t_last), 1;...
    (tmax)^3, (tmax)^2, (tmax), 1;...
    3 * (t_last)^2, 2 * (t_last), 1, 0;...
    3 * (tmax)^2, 2 * (tmax), 1, 0;];

b_start = [0;amp * omega * cos(omega * t_init);1;-amp * omega^2 * sin(omega * t_init)];
b_end = [amp * omega * cos(omega * t_last);0;-amp * omega^2 * sin(omega * t_last);-1];

C_start = A1 \ b_start;
C_end = A4 \ b_end;

t_1 = 0:0.01:t_init;
t_2 = t_last:0.01:tmax;
t_3 = t_init:0.01:t_last;
figure 
plot(t_1, polyval(C_start', t_1), 'b')
hold on
plot(t_3,f(t_3) , 'r')
plot(t_2, polyval(C_end',t_2),'b');

figure
plot(t_1, polyval(polyder(C_start'), t_1), 'b')
hold on
plot(t_3,fdash(t_3) , 'r')
plot(t_2, polyval(polyder(C_end'), t_2), 'b')