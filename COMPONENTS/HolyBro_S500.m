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


