function [k_P, k_T] = setQRS500kV()

%% Experimental Data
voltage = 16;
throttle = [0.5;0.55;0.60;0.65;0.75;0.85;1];
torque = [0.07;0.08;0.09;0.11;0.13;0.15;0.18];
thrust = [435;527;608;702;888;1076;1293]*u.gramForce/u.N;
current = [3.5;4.6;5.6;6.8;9.5;12.3;16.2];
rotor_speed = [6015;6620;7113;7563;8545;9442;10464]; % rpm

t_exp = table(throttle, torque, thrust, current, rotor_speed, 'VariableNames', ["Throttle", "Torque", "Thrust", "Current", "RotorSpeed"]);

%% Fit Torque and Speed Coeffs to Experimental Data
v_in = voltage*t_exp.Throttle;

ft = fittype({'x'});
ktau_fit = fit(t_exp.Current, t_exp.Torque,ft);
ktau = ktau_fit.a;
kV = PMSMMotor.KtTokV(ktau);

v_out = rotor_speed / kV;
R_esc = (v_in - v_out) ./ current;


end