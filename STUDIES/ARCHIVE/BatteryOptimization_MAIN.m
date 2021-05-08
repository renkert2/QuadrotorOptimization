addpath('STUDIES')
if ~(exist('batt_opt', 'var') == 1)
    batt_opt = BatteryOptimization();
end
export_path = "C:\Users\prenk\Box\ARG_Student_Reports\Philip Renkert\Weekly Reports\Renkert_WeeklyReport_02222021\";


%% Find Optimal N_p
[opt_N_p, opt_flight_time] = batt_opt.Optimize();

%% Visualize the Design Space
n_p_vals = linspace(batt_opt.N_p_bounds(1), batt_opt.N_p_bounds(2), 30);
flight_times = arrayfun(@(x) flightTime(batt_opt, x), n_p_vals);

%%
figure(1)
plot(n_p_vals, flight_times)
title("Flight Time vs. N_p - Hover")
xlabel("N_p")
ylabel("Flight Time (s)")

hold on
p = plot(opt_N_p, opt_flight_time, '.r', 'MarkerSize', 20);
datatip(p, opt_N_p, opt_flight_time);
hold off
saveas(gcf, export_path + "NpvsFlightTime_Hover_Quadrotor", 'svg')

%% Simulate at Optimum

[t, y, pf] = Simulate(batt_opt, opt_N_p);

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


