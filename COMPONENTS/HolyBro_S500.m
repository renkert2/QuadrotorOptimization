%% HolyBro S500
% https://shop.holybro.com/s500-v2-kitmotor2216-880kv-propeller1045_p1153.html

%% Frame
% The frame will contain all mass not including the battery, motors, and propellers.

% Intial calculation based on online info:
% system_mass = 0.935; % total kit mass, not including battery
% arf_mass = 0.782; % Includes Motors, Propellers, and ESCs
% esc_mass = 0.0079*4; % Approximation from: https://hobbyking.com/en_us/blheli-s-20a.html
% bare_frame_mass = 0.440;
% motorprop_mass = arf_mass - bare_frame_mass - esc_mass;
% prop_mass = 0.055/4;
% motor_mass = 0.0639;
% frame_mass = system_mass - motorprop_mass;

% Calculation based on measured values
frame_mass = 0.41455;
autopilot_mass = 0.07;
gps_mass = 0.05945;
optical_flow_mass = 0.01905; % Need to update with mount mass
rc_receiver_mass = 0.0157;

total_frame_mass = frame_mass + autopilot_mass + gps_mass + optical_flow_mass + rc_receiver_mass;

inertia_matrix = [0.00471826, 0.00000006, 0.00003197;...
    0.00000006, 0.00446811, -0.00000053;...
    0.00003197, -0.00000053, 0.00474617];

d = 0.23; % m, distance from center to rotor

frame = Frame('Name', 'Frame');
frame.Mass.Value = total_frame_mass;
frame.Mass.Tunable = true; 
frame.Mass.Unit =  "kg";
frame.J_f.Value = inertia_matrix;
frame.J_f.Unit = 'kg*m^2';
frame.d.Value = d;
frame.d.Unit = 'm';
frame.init();

%% Battery
% - Recommended: 4S, 5000 mAh.  Used 4s, 
batt = Battery('Name', 'Battery',...
    'Q', compParam('Q',4000,'Unit', 'mAh', 'AutoRename', true, 'Tunable', true),...
    'N_p', compParam('N_p',1,'Unit', 'unit', 'AutoRename', true, 'Tunable', true),...
    'N_s', compParam('N_s',4, 'Unit', 'unit', 'AutoRename', true, 'Tunable', true),... % 4000mAh, No Dynamics
    'R_s', compParam('R_s', 4e-3/4, 'Unit', "Ohm", 'AutoRename', true, 'Tunable', true),... % Measured with Battery Charger, likely not very accurate.  R_s = N_p/N_s R_p
    'Mass', extrinsicProp('Mass', 0.47735, 'Unit',"kg", 'AutoRename', true, 'Tunable', true));
 
%% ESC (Inverter)
esc = PMSMInverter('Name', 'PMSMInverter',...
    'I_max', compParam('I_max', 20, 'Unit', "A", 'AutoRename', true, 'Tunable', false),...
    'R_1', compParam('R_1', 0.01, 'Unit', "Ohm", 'AutoRename', true, 'Tunable', false),...
    'Mass', extrinsicProp('Mass', 0.026, 'Unit', "kg", 'AutoRename', true, 'Tunable', false)); % Measured
    

%% Motor
motor = PMSMMotor('Name','Motor',...
    'Mass', extrinsicProp('Mass',0.0656, 'AutoRename', true, 'Tunable', true, 'Unit', "kg"),... % Measured mass
    'J', compParam('J', NaN, 'AutoRename', true, 'Tunable', true, 'Unit', "kg*m^2"),...
    'D', compParam('D', 0.03, 'AutoRename', true, 'Tunable', true, 'Unit', "m"),... % Need to measure this
    'kV', compParam('kV', 880, 'AutoRename', true, 'Tunable', true, 'Unit', "RPM/V"),...
    'Rm', compParam('Rm',0.108, 'AutoRename', true, 'Tunable', true, 'Unit', "Ohm")); % Estimate https://www.rcmoment.com/p-rm6909.html
motor.J.Dependent = true; % Use the estimate function from PMSMMotor since we don't know the actual value
%% Propeller
D = 10*(u.in/u.m);
P = 4.5*(u.in/u.m);

prop = Propeller('Name', 'Propeller',...
    'k_P', compParam('k_P',  NaN, 'AutoRename', true, 'Tunable', true) ,... % Power coefficient - k_P = 2*pi*k_Q, speed in rev/s
    'k_T', compParam('k_T', NaN, 'AutoRename', true, 'Tunable', true),... % Thrust coefficient - N/(s^2*kg*m^2), speed in rev/s.
    'D', compParam('D', D, 'AutoRename', true, 'Tunable', true, 'Unit', "m"),...
    'P', compParam('P', P, 'AutoRename', true, 'Tunable', true, 'Unit', "m"),...
    'Mass', extrinsicProp('Mass', 0.012275, 'AutoRename',true,'Tunable',true, 'Unit', "kg"),...
    'J', compParam('J', NaN, 'AutoRename', true, 'Tunable',true, 'Unit', "kg*m^2"));

prop.J.Dependent = true; % Use the estimate function from PMSMMotor since we don't know the actual value
prop = setQRS500AeroCoeffs(prop); % Sets k_P and k_T from experimental data

%%
QR_S500 = QuadRotor('Frame', frame, 'Battery', batt, 'PMSMInverter', esc, 'Propeller', prop, 'PMSMMotor', motor);
QR_S500.Params.update();
save QR_S500.mat QR_S500