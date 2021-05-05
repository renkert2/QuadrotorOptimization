%% HolyBro S500
% https://shop.holybro.com/s500-v2-kitmotor2216-880kv-propeller1045_p1153.html

load QROpt.mat QROpt
QR_S500 = QROpt;
%% Frame
% The frame will contain all mass not including the battery, motors, and propellers.
system_mass = 0.935; % total kit mass, not including battery
arf_mass = 0.782; % Includes Motors, Propellers, and ESCs
esc_mass = 0.0079*4; % Approximation from: https://hobbyking.com/en_us/blheli-s-20a.html
bare_frame_mass = 0.440;
motorprop_mass = arf_mass - bare_frame_mass - esc_mass;
prop_mass = 0.055/4;
motor_mass = 0.0639;
frame_mass = system_mass - motorprop_mass;

QR_S500.Frame.Mass.Value = frame_mass;
QR_S500.setParams(); % Frame mass isn't tunable, requires reinitialization of system
QR_S500.init_post();

%% Battery
% - Recommended: 4S, 5000 mAh
QR_S500.Battery.N_p.Value = (5000/4000);
QR_S500.Battery.N_s.Value = 4;

% Battery Mass
% It turns out battery mass / energy decreases with pack size.
% The battery model, therefore, overpredicts mass for packs larger 
% than the nominal size and underpredicts mass for packs smaller
% than nominal size.  
%% Motor
kV = 880; % RPM / V
Rm = 0.108; % Estimate https://www.rcmoment.com/p-rm6909.html

% Estimate parameters from surrogate models
load MF_KDE.mat MF_KDE
obj.motorFit = MF_KDE;

[M_motor, J_motor, D_motor] = calcMotorProps(obj.motorFit, [kV; Rm]);
QR_S500.Motor.J.Value = J_motor;
QR_S500.Motor.kV.Value = kV;
QR_S500.Motor.M.Value = M_motor;
QR_S500.Motor.Rm.Value = Rm;
QR_S500.Motor.D.Value = D_motor;

% Actual Mass: 0.0639
% This works very well with the fit from KDE's data
%% Propeller
D = 10*(u.in/u.m);
P = 4.5*(u.in/u.m);

load PF_Aero.mat PF_Aero
obj.propAeroFit = PF_Aero;
load PF_Mass.mat PF_Mass
obj.propMassFit = PF_Mass;

[k_P_prop, k_T_prop] = calcPropCoeffs(obj.propAeroFit, [D;P]);
[M_prop,J_prop] = calcMassProps(obj.propMassFit, D);

QR_S500.Propeller.D.Value = D;
QR_S500.Propeller.P.Value = P;
QR_S500.Propeller.J.Value = J_prop;
QR_S500.Propeller.M.Value = M_prop;
QR_S500.Propeller.k_P.Value = k_P_prop;
QR_S500.Propeller.k_T.Value = k_T_prop;

% Actual mass: 0.055/4 = 0.0138 kg?
% The propeller surrogate model underestimates the k_T and k_P by a bit.
% It overestimates the mass significantly.  Predicted mass is 0.031287

%%
QR_S500.update()

%%
save QR_S500.mat QR_S500