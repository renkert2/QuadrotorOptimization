%% Init
close all
addpath(genpath('COMPONENTS'));

%% Instantiate Components

battery = Battery_Simple('Name', "Battery", 'R_s', 1);
dcbus = DCBus_CurrentEquivalence('Name', 'Bus', 'N_inputs',1,'N_outputs',1);

pmsminverter = PMSMInverter('Name', "Inverter", 'InverterType', 'VoltageDependent');
pmsmmotor = PMSMMotor('Name', "Motor");
shaftcoupler = ShaftCoupler('Name', "Shaft");
propeller = Propeller('Name', "Propeller");


%% Connect Components

components = [battery, dcbus, pmsminverter, pmsmmotor, shaftcoupler, propeller];

ConnectP = {[battery.Ports(1) dcbus.Ports(1)]};

ConnectP{end+1,1} = [pmsminverter.Ports(1), dcbus.Ports(2)];
ConnectP{end+1,1} = [pmsminverter.Ports(2), pmsmmotor.Ports(1)];
ConnectP{end+1,1} = [pmsmmotor.Ports(2), shaftcoupler.Ports(1)];
ConnectP{end+1,1} = [shaftcoupler.Ports(2), propeller.Ports(1)];

   
g_sys = Combine(components, ConnectP);
single_rotor_model = GraphModel(g_sys);

%%
sys_model = single_rotor_model;
g_sys = sys_model.graph;


%% Input Test
input_vector = 0:0.05:1.5;
disturbances = zeros(5,1);

x_all = zeros(numel(input_vector),numel(g_sys.InternalVertices));
pf_all = zeros(numel(input_vector), numel(sys_model.P_sym));

for i = 1:numel(input_vector)
    inputs = input_vector(i);
    [~, x, pf] = Simulate(single_rotor_model, inputs, disturbances, [0 1000], 'PlotStates', false, 'Solver', @ode23tb);
    
    pf_all(i,:) = pf(end,:);
    x_all(i,:) = x(end,:);
end

input_power = pf_all(:,1);
batt_power = pf_all(:,3);
bus_power = pf_all(:,5);
inv_power = pf_all(:,7);
motor_power = pf_all(:,10);
shaft_power = pf_all(:,13);

omega_final = x_all(:,5);

%% Speed 
figure
plot(input_vector, omega_final)
xlabel("Value of Input d_1")
ylabel("Propeller Speed (rad/s)")

%% System Efficiency Analysis
figure
plot(input_vector, [input_power batt_power bus_power inv_power motor_power shaft_power])
legend(["Input Power","Battery Power","Bus Power", "Inverter Power","Motor Power", "Shaft Power"])
ylabel("Power (W)")
xlabel("Value of Input d_1")

figure
plot(input_vector, [batt_power./input_power bus_power./batt_power inv_power./bus_power motor_power./inv_power shaft_power./motor_power shaft_power./input_power])
title("System Efficiency")
legend(["$$\eta_{battery}$$","$$\eta_{bus}$$", "$$\eta_{inverter}$$","$$\eta_{motor}$$", "$$\eta_{shaft}$$", "$$\eta_{overall}$$"], 'Interpreter', 'latex')
ylabel("$$\eta$$", 'Interpreter', 'latex')
xlabel("Value of Input d_1")

%% Inverter Efficiency
figure
plot(x_all(:,6), inv_power./bus_power)

%% Optimization

% Let's find the optimal throttle input for
% 1) System efficiency
% 2) Maximum Speed

d_opt_eta = fminbnd(@(d) (1-systemEfficiency(single_rotor_model,d)), 0, 2);
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
inputs = @(t) d_opt_omega.*((0<t)&(t<=5))+d_opt_eta.*((5<t)&(t<=40))+0.65.*(t>40);
disturbances = zeros(5,1);

figure
t = 0:0.1:60;
plot(t, inputs(t))
title("Mission Input Profile")
xlabel("t (sec)")
ylabel("Inverter Input d")

[t,x] = Simulate(single_rotor_model, inputs, disturbances, [0 60], 'Solver', @ode23tb, 'PlotStates',false);
omega = x(:,5);
thrust = propeller.square_thrust_coeff.*((omega./(2*pi)).^2);
inv_current = x(:,8);

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
    [~,~, pf] = Simulate(model, d, disturbances, [0 1000], 'PlotStates', false, 'Solver', @ode23tb);
    input_power = pf(end,1);
    shaft_power = pf(end,13);
    sys_eta = shaft_power./input_power;
end

function shaft_speed = systemSpeed(model, d)
    disturbances = zeros(5,1);
    [~,x, ~] = Simulate(model, d, disturbances, [0 1000], 'PlotStates', false, 'Solver', @ode23tb);
    shaft_speed = x(end,5);
end


    