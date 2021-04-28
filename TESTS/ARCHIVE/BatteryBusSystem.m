%% Init
close all
addpath(genpath('COMPONENTS'));

%% Instantiate Components

battery = Battery('Name', "Battery");
dcbus = DCBus_CurrentEquivalence('Name', 'Bus', 'N_inputs',1,'N_outputs',1);

%% Connect Components
components = [battery, dcbus];

ConnectP = {[battery.Ports(1) dcbus.Ports(1)]};

g_sys = Combine(components, ConnectP);
sys_model = GraphModel(g_sys);

%% System Simulation Init
inputs = [0];
disturbances = zeros(3,1);
disturbances(2) = 1; % Set constant current output

%% Simulation
[t,x] = Simulate(sys_model, inputs, disturbances, [], [0 700], 'PlotStates', false, 'Solver', @ode23tb);


%% Plots
bus_voltage = x(:,4);
batt_soc = x(:,1);

plot(t,bus_voltage)
