%%
rt = ReferenceTrajectory3D('Lemniscate');
%%
t = rt.t;
data = rt.R_Data;
accel_data = rt.ddR_Data;



plot(t,data)
hold on
plot(t,accel_data)
hold off

%%
vel_data_i = vel_data(:, 1) + cumtrapz(t, accel_data, 2);
pos_data_i = cumtrapz(t, vel_data, 2);

plot(t,data)
hold on
plot(t,pos_data_i)
hold off
%% Int Der Test

f = @(x) x.^2 + x;

dX = 0.01;
x = 0:dX:1;
y = f(x);

% First Derivative
dy = [diff(y)./diff(x) 0];

% Second Derivative
ddy = [diff(dy)./diff(x) 0];

% First Integral
iddy = cumtrapz(x, ddy, 2) + dy(1); 

% Second Integral
iiddy = cumtrapz(x,iddy,2);

%%
plot(x,iiddy)
hold on
plot(x,y)
hold off
