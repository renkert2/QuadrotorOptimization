%% Init
close all
addpath(genpath('COMPONENTS'));

%% Instantiate Components

battery = Battery('Name', "Battery");
dcbus = DCBus_CurrentEquivalence('Name', 'Bus', 'N_inputs',1,'N_outputs',4);

for i = 1:4 % Each of the following components occur four times in a quadrotor
    pmsminverter(i) = PMSMInverter('Name', sprintf("Inverter %d",i));
    pmsmmotor(i) = PMSMMotor('Name', sprintf("Motor %d", i));
    shaftcoupler(i) = ShaftCoupler('Name', sprintf("Shaft %d",i));
    propeller(i) = Propeller('Name', sprintf("Prop %d",i));
end

%% Connect Components

components = [battery, dcbus, pmsminverter, pmsmmotor, shaftcoupler, propeller];

ConnectP = {[battery.Ports(1) dcbus.Ports(1)]};

for i = 1:4
    ConnectP{end+1,1} = [dcbus.Ports(1+i), pmsminverter(i).Ports(1)];
    ConnectP{end+1,1} = [pmsminverter(i).Ports(2), pmsmmotor(i).Ports(1)];
    ConnectP{end+1,1} = [pmsmmotor(i).Ports(2), shaftcoupler(i).Ports(1)];
    ConnectP{end+1,1} = [shaftcoupler(i).Ports(2), propeller(i).Ports(1)];
end
   
g_sys = Combine(components, ConnectP);
sys_model = GraphModel(g_sys)

%% Plot Graph
plot(sys_model)

%% System Tables
% Vertex Table
names = vertcat(g_sys.Vertices.Description);
parents = vertcat(vertcat(g_sys.Vertices.Parent).Name);
types = vertcat(g_sys.Vertices.VertexType);
vertex_table = table((1:(g_sys.Nv+g_sys.Nev))', parents, names, types, 'VariableNames', ["Vertices", "Component", "Description", "Domain"])

% Edge Table
digits(4)
pflows = vpa(sys_model.P_sym);
pflows_strings = arrayfun(@string, pflows);
digits(32)

parents = vertcat(vertcat(g_sys.InternalEdges.Parent).Name);

edge_table = table((1:(g_sys.Ne - g_sys.Nee))',parents, pflows_strings, 'VariableNames', ["Edges", "Component", "PowerFlows"])

%% System Simulation

sys_model.StateNames
sys_model.InputNames
sys_model.DisturbanceNames

inputs = 1*ones(4,1);
disturbances = zeros(14,1);

%[t,x] = Simulate(sys_model, inputs, disturbances, [0 10], 'StateSelect', [1,13]);
Simulate(sys_model, inputs, disturbances, [0 200], 'StateSelect', [24]);
title("Speed of Propeller 1 (Step Input)")
xlabel('t (sec)')
ylabel('Propeller Speed (rad/s)')

%%
Simulate(sys_model, inputs, disturbances, [0 200], 'StateSelect', [1]);
title("Battery State of Charge (SOC)")
xlabel('t (sec)')
ylabel('SOC')

%% Input Test
% input_vector = 0.1:0.1:3;
% omega_final = zeros(size(input_vector));
% for i = 1:numel(input_vector)
%     inputs = input_vector(i).*ones(4,1);
%     [t,x] = Simulate(sys_model, inputs, disturbances, [0 1], 'PlotStates', false);
%     omega_final(i) = x(end,24);
% end
% %%
% plot(input_vector, omega_final)
% ylabel("Propeller Speed at t = 1")
% xlabel("Value of Input d_1")