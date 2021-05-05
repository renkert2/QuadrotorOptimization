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
frame_mass = system_mass - motorprop_mass;

QR_S500.Frame.Mass.Value = frame_mass;
QR_S500.setParams(); % Frame mass isn't tunable, requires reinitialization of system
QR_S500.init_post();

%% Battery
% - Recommended: 4S, 5000 mAh

%% Motor
kV = 880; % RPM / V
Rm = 0.108; % Estimate https://www.rcmoment.com/p-rm6909.html

% Estimate parameters from surrogate models

% Actual Mass: 0.039 kg
%% Propeller
D = 10*(u.in/u.m);
P = 4.5*(u.in/u.m);

% Estimate prop params and update tunable params of the prop

% Actual mass: 0.055/4 kg
