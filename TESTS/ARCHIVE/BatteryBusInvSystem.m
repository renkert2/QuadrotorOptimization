%% Init
close all
addpath(genpath('COMPONENTS'));

%% Instantiate Components

battery = Battery('Name', "Battery");
battery_lossless = Battery_Simple('Name', "Battery_Lossless", 'R_s', 0);
dcbus = DCBus_CurrentEquivalence('Name', 'Bus', 'N_inputs',1,'N_outputs',1);
pmsminverter = PMSMInverter('Name', "Inverter");

%% Connect Components
components = [battery, dcbus, pmsminverter];

ConnectP = {[battery.Ports(1) dcbus.Ports(1)];
    [pmsminverter.Ports(1), dcbus.Ports(2)]};

g_sys = Combine(components, ConnectP);
sys_model = GraphModel(g_sys);

components_lossless = [battery_lossless, dcbus, pmsminverter];

ConnectP_lossless = {[battery_lossless.Ports(1) dcbus.Ports(1)];
    [pmsminverter.Ports(1), dcbus.Ports(2)]};

g_sys_lossless = Combine(components_lossless, ConnectP_lossless);
sys_model_lossless = GraphModel(g_sys_lossless);

%% System Simulation Init
inputs = 1;
disturbances = zeros(4,1);
disturbances(3) = 15;

%% Input Test
input_vector = 0:0.1:2;
current_vector = 0:.1:10;
disturbances = zeros(4,1);
power = zeros(numel(input_vector),numel(current_vector));
power_lossless = power;
for i = 1:numel(input_vector)
    for j = 1:numel(current_vector)
        inputs = input_vector(i);
        disturbances(3) = current_vector(j);
        [t,x] = Simulate(sys_model, inputs, disturbances, [0 1000], 'PlotStates', false, 'Solver', @ode23tb);
        [t_lossless,x_lossless] = Simulate(sys_model_lossless, inputs, disturbances, [0 1000], 'PlotStates', false, 'Solver', @ode23tb);
        power(i,j) = x(end,7).*current_vector(j);
        power_lossless(i,j) = x_lossless(end,5).*current_vector(j);
    end
end

%%
figure
surf(input_vector, current_vector, power')
zlim([0 100])
xlabel("Input d")
ylabel("Output Current")
zlabel("Output Power")

figure
surf(input_vector, current_vector, power_lossless')

zlim([0 100])
xlabel("Input d")
ylabel("Output Current")
zlabel("Output Power")