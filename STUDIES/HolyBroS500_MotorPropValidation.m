load QR_S500.mat;
%%
prop = QR_S500.Propeller;
motor = QR_S500.Motor;

% prop_default = getValues(prop.Params);
% motor_default = getValues(motor.Params);

%% Experimental Data
throttle = [0.5;0.55;0.60;0.65;0.75;0.85;1];
torque = [0.07;0.08;0.09;0.11;0.13;0.15;0.18];
thrust = [435;527;608;702;888;1076;1293]*u.gramForce/u.N;
current = [3.5;4.6;5.6;6.8;9.5;12.3;16.2];
rotor_speed = [6015;6620;7113;7563;8545;9442;10464]*u.revolutionPerMinute/(u.radian/u.second);

t_exp = table(throttle, torque, thrust, current, rotor_speed, 'VariableNames', ["Throttle", "Torque", "Thrust", "Current", "RotorSpeed"]);

makeAndPlot(QR_S500,t_exp)
%% Fit Torque and Speed Coeffs to Experimental Data
ft = fittype({'x^2'});
torque_fit = fit(t_exp.RotorSpeed,t_exp.Torque,ft);
thrust_fit = fit(t_exp.RotorSpeed,t_exp.Thrust,ft);

prop.D.Tunable = false;
K_T = Propeller.convCoeffToRevPerS(thrust_fit.a); % lumped rev/s
K_Q = Propeller.convCoeffToRevPerS(torque_fit.a); % lumped rev/s

k_T = prop.lumpedToThrustCoeff(K_T);
k_Q = prop.lumpedToTorqueCoeff(K_Q);

k_P = k_Q * 2*pi;
prop.D.Tunable = true;

prop.k_P.Value = k_P;
prop.k_T.Value = k_T;

makeAndPlot(QR_S500,t_exp)
%% Fit Motor Values to Experimental Data
ft = fittype({'x'});
kt_fit = fit(t_exp.Current, t_exp.Torque, ft);
kt = kt_fit.a;
kv = PMSMMotor.KtTokV(kt);
motor.kV.Value = kv;

makeAndPlot(QR_S500,t_exp)
%% 
function makeAndPlot(QR_S500, t_exp)
inv = QR_S500.Inverter;
motorprop = QR_S500.Components([QR_S500.Components.Name] == "MotorProp_1");

conn_p = {[inv.Ports(2), motorprop.Ports(1)]};

imp = Combine([inv; motorprop], conn_p);



% Model Data
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

% Plots
figure

% Thrust
subplot(2,2,1)
plot(t_model.RotorSpeed, t_model.Thrust, '-b')
hold on
plot(t_exp.RotorSpeed, t_exp.Thrust, '.r');
hold off
title("Thrust vs Rotor Speed")
xlabel("$$\omega$$ (rads/s)", 'Interpreter', 'latex')
ylabel("$$T$$ (N)", 'Interpreter', 'latex')
legend(["Predicted", "Measured"]);

% Torque
subplot(2,2,3)
plot(t_model.RotorSpeed, t_model.Torque, '-b')
hold on
plot(t_exp.RotorSpeed, t_exp.Torque, '.r');
hold off
title("Torque vs Rotor Speed")
xlabel("$$\omega$$ (rads/s)", 'Interpreter', 'latex')
ylabel("$$ \tau $$ (N*m)", 'Interpreter', 'latex')
legend(["Predicted", "Measured"]);

% Torque vs Current
subplot(2,2,2)
plot(t_model.Current, t_model.Torque, '-b')
hold on
plot(t_exp.Current, t_exp.Torque, '.r');
hold off
title("Torque vs Current")
xlabel("$$I$$ (A)", 'Interpreter', 'latex')
ylabel("$$ \tau $$ (N*m)", 'Interpreter', 'latex')
legend(["Predicted", "Measured"]);


% Speed vs Current
subplot(2,2,4)
plot(t_model.Current, t_model.RotorSpeed, '-b')
hold on
plot(t_exp.Current, t_exp.RotorSpeed, '.r');
hold off
title("Rotor Speed vs Current")
xlabel("$$I$$ (A)", 'Interpreter', 'latex')
ylabel("$$\omega$$ (rads/s)", 'Interpreter', 'latex')
legend(["Predicted", "Measured"]);
end
