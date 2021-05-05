load QR_S500.mat;

%% 
inv = QR_S500.Inverter;
motorprop = QR_S500.Components([QR_S500.Components.Name] == "MotorProp_1");

conn_p = {[inv.Ports(2), motorprop.Ports(1)]};

imp = Combine([inv; motorprop], conn_p);

%% Model Data
bus_voltage = 16;
d = [bus_voltage;0;0;0]; % Disturbance Vector

u_vals = linspace(0.1,1,100);
t_model = table('Size', [numel(u_vals) 5], 'VariableTypes',repmat("double", [1 5]), 'VariableNames', ["Throttle", "Torque", "Thrust", "Current", "RotorSpeed"]);
for i = 1:numel(u_vals)
    [~,y_bar] = calcSteadyState(imp.Model, u_vals(i), [16;0;0;0]);
    t_model.Throttle(i) = u_vals(i);
    t_model.Torque(i) = y_bar(6);
    t_model.Thrust(i) = y_bar(5);
    t_model.Current(i) = y_bar(3);
    t_model.RotorSpeed(i) = y_bar(2);
end

%% Experimental Data
throttle = [0.5;0.55;0.60;0.65;0.75;0.85;1];
torque = [0.07;0.08;0.09;0.11;0.13;0.15;0.18];
thrust = [435;527;608;702;888;1076;1293]*u.gramForce/u.N;
current = [3.5;4.6;5.6;6.8;9.5;12.3;16.2];
rotor_speed = [6015;6620;7113;7563;8545;9442;10464]*u.revolutionPerMinute/(u.radian/u.second);

t_exp = table(throttle, torque, thrust, current, rotor_speed, 'VariableNames', ["Throttle", "Torque", "Thrust", "Current", "RotorSpeed"]);
    

%% Plots

% Speed vs Current
figure
plot(t_model.Current, t_model.RotorSpeed, '-b')
hold on
plot(t_exp.Current, t_exp.RotorSpeed, '.r');
hold off
title("Rotor Speed vs Current")
xlabel("$$I$$ (A)", 'Interpreter', 'latex')
ylabel("$$\omega$$ (rads/s)", 'Interpreter', 'latex')
legend(["Predicted", "Measured"]);


%%
% Thrust
figure
plot(t_model.RotorSpeed, t_model.Thrust, '-b')
hold on
plot(t_exp.RotorSpeed, t_exp.Thrust, '.r');
hold off
title("Thrust vs Rotor Speed")
xlabel("$$\omega$$ (rads/s)", 'Interpreter', 'latex')
ylabel("$$T$$ (N)", 'Interpreter', 'latex')
legend(["Predicted", "Measured"]);

% Torque
figure
plot(t_model.RotorSpeed, t_model.Torque, '-b')
hold on
plot(t_exp.RotorSpeed, t_exp.Torque, '.r');
hold off
title("Torque vs Rotor Speed")
xlabel("$$\omega$$ (rads/s)", 'Interpreter', 'latex')
ylabel("$$ \tau $$ (N*m)", 'Interpreter', 'latex')
legend(["Predicted", "Measured"]);

%%
% Torque vs Current
figure
plot(t_model.Current, t_model.Torque, '-b')
hold on
plot(t_exp.Current, t_exp.Torque, '.r');
hold off
title("Torque vs Current")
xlabel("$$I$$ (A)", 'Interpreter', 'latex')
ylabel("$$ \tau $$ (N*m)", 'Interpreter', 'latex')
legend(["Predicted", "Measured"]);

%%
% Speed vs Torque
figure
plot(t_model.Torque, t_model.RotorSpeed, '-b')
hold on
plot(t_exp.Torque, t_exp.RotorSpeed, '.r');
hold off
title("Rotor Speed vs Torque")
xlabel("$$\tau$$ (N*m)", 'Interpreter', 'latex')
ylabel("$$\omega$$ (rads/s)", 'Interpreter', 'latex')
legend(["Predicted", "Measured"]);