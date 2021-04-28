%% Single-Rotor System: Efficiency Analysis, Simulation, and Basic Optimization
run SingleRotorSystem
sys_model = single_rotor_model;
g_sys = sys_model.graph;

%% Plot Graph
figure
plot(single_rotor_model);
title("Single-Rotor System Graph Model");

%% Vertex Table
names = vertcat(g_sys.Vertices.Description);
parents = vertcat(vertcat(g_sys.Vertices.Parent).Name);
types = vertcat(g_sys.Vertices.VertexType);
vertex_table = table((1:(g_sys.Nv+g_sys.Nev))', parents, names, types, 'VariableNames', ["Vertices", "Component", "Description", "Domain"])

%% Edge Table
digits(4)
pflows = vpa(single_rotor_model.P_sym);
pflows_strings = arrayfun(@string, pflows);
digits(32)

parents = vertcat(vertcat(g_sys.InternalEdges.Parent).Name);

edge_table = table((1:(g_sys.Ne - g_sys.Nee))',parents, pflows_strings, 'VariableNames', ["Edges", "Component", "PowerFlows"])

%% Input Test
input_vector = 0:0.05:1.5;
disturbances = zeros(5,1);

input_power = zeros(size(input_vector));
batt_power = zeros(size(input_vector));
bus_power = zeros(size(input_vector));
inv_power = zeros(size(input_vector));
inv_current = zeros(size(input_vector));
motor_power = zeros(size(input_vector));
shaft_power = zeros(size(input_vector));

omega_final = zeros(size(input_vector));

elec_loss = zeros(size(input_vector));
mech_loss = zeros(size(input_vector));

x_all = zeros(length(input_vector),numel(g_sys.InternalVertices));

for i = 1:numel(input_vector)
    inputs = input_vector(i);
    [t,x,pf] = Simulate(single_rotor_model, inputs, disturbances,[], [0 1000], 'PlotStates', false, 'Solver', @ode23tb);

    input_power(i) = 11.1*x(end,9);
    batt_power(i) = (x(end,8).*x(end,9));
    bus_power(i) = 1.225*inputs*x(end,8).*x(end,10);
    inv_power(i) = 1.225*inputs*x(end,11)*x(end,4);
    inv_current(i) = x(end,10);
    motor_power(i) = x(end,5)*x(end,6);
    shaft_power(i) = x(end,6).*x(end,7);
    
    omega_final(i) = x(end,7);
    
    elec_loss(i) = 0.117.*x(end,4).^2; % Motor i^2*r loss
    mech_loss(i) = 2.415e-6.*x(end,5).^2; % Motor w^2*B loss
    
    x_all(i,:) = x(end,:);
end

%% Speed vs. Input
figure
plot(input_vector, omega_final)
xlabel("Value of Input d_1")
ylabel("Propeller Speed (rad/s)")

%% System Efficiency Analysis
figure
plot(input_vector, [input_power' batt_power' bus_power' inv_power' motor_power' shaft_power'])
legend(["Input Power","Battery Power","Bus Power", "Inverter Power","Motor Power", "Shaft Power"])
ylabel("Power (W)")
xlabel("Value of Input d_1")

figure
plot(input_vector, [batt_power'./input_power' bus_power'./batt_power' inv_power'./bus_power' motor_power'./inv_power' shaft_power'./motor_power' shaft_power'./input_power'])
title("System Efficiency")
legend(["$$\eta_{battery}$$","$$\eta_{bus}$$", "$$\eta_{inverter}$$","$$\eta_{motor}$$", "$$\eta_{shaft}$$", "$$\eta_{overall}$$"], 'Interpreter', 'latex')
ylabel("$$\eta$$", 'Interpreter', 'latex')
xlabel("Value of Input d_1")

%% Inverter Efficiency
figure
plot(input_vector, x_all(:, [8 10]))
title("Inverter Electrical Input")
legend(["DC Voltage", "DC Current"])
xlabel("Value of Input d_1")

figure
plot(input_vector, x_all(:,[11 4]))
title("Inverter Electrical Output (q-Phase)")
legend(["Q Voltage", "Q Current"])
xlabel("Value of Input d_1")

%% Motor Losses
figure
plot(input_vector, [inv_power' elec_loss' mech_loss' motor_power'])
title('Motor Losses')
legend(["Motor Input Power", "Electrical Loss", "Mechanical Loss", "Motor Output Power"])
ylabel("Power (W)")
xlabel("Value of Input d_1")

%% Basic Optimization
% Find the optimal throttle input for
%
% * Maximum system efficiency
% * Maximum speed
%

% Input required to maximize system efficiency
d_opt_eta = fminbnd(@(d) (1-systemEfficiency(single_rotor_model,d)), 0, 2);

% Input required to maximize propeller speed (thrust)
d_opt_omega = fminbnd(@(d) -systemSpeed(single_rotor_model,d), 0, 2);

figure
subplot(1,2,1)
p = plot(input_vector, omega_final);
datatip(p,d_opt_omega, systemSpeed(single_rotor_model, d_opt_omega));
title("Optimal Input for Propeller Speed")
xlabel("d")
ylabel("rad/s")
xline(d_opt_omega)

subplot(1,2,2)
p=plot(input_vector, shaft_power./input_power);
datatip(p,d_opt_eta, systemEfficiency(single_rotor_model, d_opt_eta));
title("Optimal Input for System Efficiency")
xlabel("d")
ylabel("$$\eta_{overall}$$",'Interpreter', 'latex')
xline(d_opt_eta)

%% System Simulation
% Simulate the system over a simple mission with three phases:
% 
% # Takeoff: Input at thrust-maximizing level
% # Cruise: Input at efficiency-maximizing level
% # Landing: Input slightly less than that required to hover
%

inputs = @(t) d_opt_omega.*((0<t)&(t<=5))+d_opt_eta.*((5<t)&(t<=40))+0.65.*(t>40);
disturbances = zeros(5,1);

figure
t = 0:0.1:60;
plot(t, inputs(t))
title("Mission Input Profile")
xlabel("t (sec)")
ylabel("Inverter Input d")

[t,x] = Simulate(single_rotor_model, inputs, disturbances,[], [0 60], 'Solver', @ode23tb, 'PlotStates',false);
omega = x(:,7);
thrust = propeller.square_thrust_coeff.*((omega./(2*pi)).^2);
inv_current = x(:,10);

figure
subplot(1,3,1)
plot(t,thrust)
title("Propeller Thrust")
xlabel('t (sec)')
ylabel('Propeller Thrust (N)')

subplot(1,3,2)
plot(t,omega)
title("Propeller Speed")
xlabel('t (sec)')
ylabel('rad/s')

subplot(1,3,3)
plot(t,inv_current)
title("Inverter Current")
xlabel('t (sec)')
ylabel('A')

%% Optimization Functions
function sys_eta = systemEfficiency(model, d)
    disturbances = zeros(5,1);
    [~,x] = Simulate(model, d, disturbances,[], [0 1000], 'PlotStates', false, 'Solver', @ode23tb);
    input_power = 11.1*x(end,9);
    shaft_power = x(end,6).*x(end,7);
    sys_eta = shaft_power./input_power;
end

function shaft_speed = systemSpeed(model, d)
    disturbances = zeros(5,1);
    [~,x] = Simulate(model, d, disturbances,[], [0 1000], 'PlotStates', false, 'Solver', @ode23tb);
    shaft_speed = x(end,7);
end


    