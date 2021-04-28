addpath('STUDIES')
if ~(exist('po', 'var') == 1)
    po = PropOptimization();
end
%%
export_path = "C:\Users\prenk\Box\ARG_Student_Reports\Philip Renkert\Weekly Reports\Renkert_WeeklyReport_03012021\";

%% Feasible Domain
[d,p,grid] = validDomain(po, 30);
bound_points = po.propFit.Boundary.points;
%%
figure(1)
surf(d,p,grid);
hold on
plot3(bound_points(:,1), bound_points(:,2), zeros(size(bound_points,1),1),"-r")
hold off
title("Feasible Domain")
xlabel("Diameter (m)")
ylabel("Pitch (m)")
zlabel("Distance from Boundary")

%% Find Optimal N_p
opts = optimoptions('fmincon','Algorithm', 'sqp', 'FiniteDifferenceType', 'central', 'FiniteDifferenceStepSize', 0.01, 'Diagnostics','on', 'Display', 'iter-detailed');
% Not able to find true optimum - needs more debugging
po.D_init = 0.4;
po.P_init = 0.2;
[opt_X, opt_flight_time] = Optimize(po, 'OptimOptions', opts);

%% Visualize the Design Space
[d_vals, p_vals] = po.validDomain(30);
N_d_vals = numel(d_vals);
N_p_vals = numel(p_vals);

%%
flight_times = zeros(N_d_vals, N_p_vals);
for i = 1:N_d_vals
    d_val = d_vals(i);
    for j = 1:N_p_vals
        p_val = p_vals(j);
        if po.propFit.Boundary.distance_func(d_val, p_val) < 0
            x = [d_val; p_val];  
            flight_times(j,i) =  flightTime(po, x);
        else
            flight_times(j,i) = NaN;
        end
    end
end

%% D-Sweep
opt_X_dsweep = zeros(2,numel(d_vals)-3);
opt_flight_time_dsweep = zeros(1,size(opt_X_dsweep,1));
opts = optimoptions('fmincon','Algorithm', 'sqp','FiniteDifferenceType', 'central', 'FiniteDifferenceStepSize', 0.01);
for i = 2:(N_d_vals-2)
    d_val = d_vals(i);
    try
        [opt_X_dsweep(:,i), opt_flight_time_dsweep(i)] = Optimize(po, 'FixDiameter', d_val, 'OptimOptions', opts);
    catch
        opt_X_dsweep(:,i) = [d_val, NaN];
        opt_flight_time_dsweep(i) = NaN;
    end
end

%%
figure(1)
surf(d_vals, p_vals, flight_times)
title("Flight Time - Hover")
xlabel("D")
ylabel("P")
zlabel("Flight Time (s)")
hold on
% p = plot3(opt_X(1), opt_X(2), opt_flight_time, '.r', 'MarkerSize', 20);
% datatip(p, opt_X(1), opt_X(2), opt_flight_time);
plot3(opt_X_dsweep(1,:), opt_X_dsweep(2,:), opt_flight_time_dsweep, '-r', 'LineWidth', 2)
hold off
%saveas(gcf, export_path + "PDvsFlightTime_Hover_Quadrotor_DatatipAndLine", 'svg')

%%
figure(2)
plot(opt_X_dsweep(1,:), opt_flight_time_dsweep);
title("Flight Time vs. Diameter - Optimal Pitch")
xlabel('Diameter (m)')
ylabel('Flight Time (s)')
saveas(gcf, export_path + "FlightTime_vs_D_OptPitch", 'svg')

%%

figure(3)
hold on
plot(opt_X_dsweep(1,:), opt_X_dsweep(2,:))
hold off

%% Compare Results to Commercially Available Drones
T = readtable("CommercialDrones.xlsx", 'TextType', 'string');
figure(2)
hold on
scatter(T.Prop_Diameter_m, T.Flight_Time_s)
hold off



%% Simulate at Optimum
d_val = 0.182; % Mavic Air II Prop Diameter
opts = optimoptions('fmincon','Algorithm', 'sqp','FiniteDifferenceType', 'central', 'FiniteDifferenceStepSize', 0.01);
[opt_X, opt_ft] = Optimize(po, 'FixDiameter', d_val, 'OptimOptions', opts);
[t, y, pf] = Simulate(po, opt_X);

%% Simulate Other
u = symunit;
X = double(separateUnits(unitConvert([9.5; 5]*u.in, u.m)));
u = po.calcInputSchedule(X);

[t, y, pf] = Simulate(po, X);
%%
batt_soc = y(:,1);
motor_current = y(:,4);
motor_current_2 = y(:,6);
rotor_speed = y(:,16);
rotor_speed_2 = y(:,17);
bus_voltage = y(:,20);
bus_current = y(:,21);

% Powers going Out of component
input_power = pf(:,1);
batt_power = pf(:,7);
bus_power = 4*pf(:,9);
inv_power = 4*pf(:,11);
motor_power = 4*pf(:,26);
shaft_power = 4*pf(:,41);



%% Quick Look at Efficiencies
figure(1)
plot(t, [input_power batt_power bus_power inv_power motor_power shaft_power])
legend(["Input Power","Battery Power","Bus Power", "Inverter Power","Motor Power", "Shaft Power"])
ylabel("Power (W)")
xlabel("t")

figure(2)
plot(t, [batt_power./input_power bus_power./batt_power inv_power./bus_power motor_power./inv_power shaft_power./motor_power shaft_power./input_power])
title("System Efficiency")
legend(["$$\eta_{battery}$$","$$\eta_{bus}$$", "$$\eta_{inverter}$$","$$\eta_{motor}$$", "$$\eta_{shaft}$$", "$$\eta_{overall}$$"], 'Interpreter', 'latex')
ylabel("$$\eta$$", 'Interpreter', 'latex')
xlabel("t")


