addpath('STUDIES')
if ~(exist('batt_opt', 'var') == 1)
    batt_opt = BatteryOptimization();
end
export_path = "C:\Users\prenk\Box\ARG_Student_Reports\Philip Renkert\Weekly Reports\Renkert_WeeklyReport_02152021\EXPORT\";


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

%[t,y] = Simulate(batt_opt, opt_N_p);
%[t,y] = Simulate(batt_opt, 1);

%% Adjust Input Schedule

% Each element of mission_thrust_factor k determines the input
% required such that the thrust produced is k*(m*g), or k times 
% the thrust required to hover. Resulting vertical acceleration is 
% a = g(k-1)

batt_opt.mission_thrust_factor = [2 0.5 2 0.5 1];

% mission_thrust_times is the time at which the corresponding 
% mission_thrust_factor begins
batt_opt.mission_thrust_times = [0 7500 10000 17500 20000];

figure(3)
vals = [0 batt_opt.mission_thrust_factor];
thrust_schedule = @(t) vals(sum(t>=batt_opt.mission_thrust_times.') + 1);
fplot(thrust_schedule, [0 40000])
title("Thrust Schedule: $$T = kT_{hover}$$", 'Interpreter', 'latex')
xlabel("Time (s)")
ylabel("$$k$$", 'Interpreter', 'latex')

%%
in_schedule = batt_opt.calcInputSchedule(1);

figure(2)
fplot(in_schedule, [0 40000])
title("Input Schedule for N_p = 1")
xlabel("Time (s)")
ylabel("Input Value (d)")
saveas(gcf, export_path + "ExampleMission", 'svg')
%% Find Optimal N_p
[opt_N_p, opt_flight_time] = batt_opt.Optimize();


%% Visualize the Design Space
n_p_vals = linspace(batt_opt.N_p_bounds(1), batt_opt.N_p_bounds(2), 25);
flight_times = arrayfun(@(x) flightTime(batt_opt, x), n_p_vals);

%%
figure(3)
plot(n_p_vals, flight_times)
title("Flight Time vs. N_p - Mission")
xlabel("N_p")
ylabel("Flight Time (s)")

hold on
p = plot(opt_N_p, opt_flight_time, '.r', 'MarkerSize', 20);
datatip(p, opt_N_p, opt_flight_time);
hold off
saveas(gcf, export_path + "NpvsFlightTime_Mission", 'svg')
