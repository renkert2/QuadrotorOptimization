function [prop, k_P, k_T] = setQRS500AeroCoeffs(prop)

%% Experimental Data
throttle = [0.5;0.55;0.60;0.65;0.75;0.85;1];
torque = [0.07;0.08;0.09;0.11;0.13;0.15;0.18];
thrust = [435;527;608;702;888;1076;1293]*u.gramForce/u.N;
current = [3.5;4.6;5.6;6.8;9.5;12.3;16.2];
rotor_speed = [6015;6620;7113;7563;8545;9442;10464]*u.revolutionPerMinute/(u.radian/u.second);

t_exp = table(throttle, torque, thrust, current, rotor_speed, 'VariableNames', ["Throttle", "Torque", "Thrust", "Current", "RotorSpeed"]);

%% Fit Torque and Speed Coeffs to Experimental Data
ft = fittype({'x^2'});
torque_fit = fit(t_exp.RotorSpeed,t_exp.Torque,ft);
thrust_fit = fit(t_exp.RotorSpeed,t_exp.Thrust,ft);

D_tunable_cache = prop.D.Tunable;
prop.D.Tunable = false;
K_T = Propeller.convCoeffToRevPerS(thrust_fit.a); % lumped rev/s
K_Q = Propeller.convCoeffToRevPerS(torque_fit.a); % lumped rev/s

k_T = prop.lumpedToThrustCoeff(K_T);
k_Q = prop.lumpedToTorqueCoeff(K_Q);

k_P = k_Q * 2*pi;
prop.D.Tunable = D_tunable_cache;

prop.k_P.Value = k_P;
prop.k_T.Value = k_T;

end