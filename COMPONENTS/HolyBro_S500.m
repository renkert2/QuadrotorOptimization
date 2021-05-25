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

%% Battery
% - Recommended: 4S, 5000 mAh
QR_S500.Battery.Q.Value = 5000;
QR_S500.Battery.N_s.Value = 4;

% Battery Mass
% It turns out battery mass / energy decreases with pack size.
% The battery model, therefore, overpredicts mass for packs larger 
% than the nominal size and underpredicts mass for packs smaller
% than nominal size.  
%% Motor
kV = 880; % RPM / V
Rm = 0.108; % Estimate https://www.rcmoment.com/p-rm6909.html

QR_S500.Motor.kV.Value = kV;
QR_S500.Motor.Rm.Value = Rm;

% Actual Mass: 0.0639
% This works very well with the fit from KDE's data
%% Propeller
D = 10*(u.in/u.m);
P = 4.5*(u.in/u.m);

QR_S500.Propeller.D.Value = D;
QR_S500.Propeller.P.Value = P;

% Actual mass: 0.055/4 = 0.0138 kg?
% The propeller surrogate model underestimates the k_T and k_P by a bit.
% It overestimates the mass significantly.  Predicted mass is 0.031287

%%
QR_S500.update() % Automatically updates parameters

%%
save QR_S500.mat QR_S500