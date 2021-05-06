frame = Frame('Name', 'Frame', 'Mass', extrinsicProp("Mass", 0.284 - 0.080 - 4*0.008 - 4*0.04, 'AutoRename', true, 'Tunable', true, 'Unit', "kg"));

batt = Battery('Name', 'Battery',...
    'N_p', compParam('N_p',1,'AutoRename', true, 'Tunable', true),...
    'N_s', compParam('N_s',6, 'AutoRename', true, 'Tunable', true)); % 4000mAh, 6S Default Battery, No Dynamics

prop = Propeller('Name', 'Propeller',...
    'k_P', compParam('k_P',  0.0411, 'AutoRename', true, 'Tunable', true) ,... % Power coefficient - k_P = 2*pi*k_Q, speed in rev/s
    'k_T', compParam('k_T', 0.0819, 'AutoRename', true, 'Tunable', true),... % Thrust coefficient - N/(s^2*kg*m^2), speed in rev/s.
    'D', compParam('D', 0.1780, 'AutoRename', true, 'Tunable', true, 'Unit', "m"),...
    'P', compParam('P', 0.0673, 'AutoRename', true, 'Tunable', true, 'Unit', "m"),...
    'M', extrinsicProp('Mass', 0.008, 'AutoRename',true,'Tunable',true, 'Unit', "kg"),...
    'J', compParam('J', 2.1075e-05, 'AutoRename', true, 'Tunable',true, 'Unit', "kg*m^2"));

motor = PMSMMotor('Name','Motor',...
    'M', extrinsicProp('Mass',0.04, 'AutoRename', true, 'Tunable', true, 'Unit', "kg"),...
    'J', compParam('J', 6.5e-6, 'AutoRename', true, 'Tunable', true, 'Unit', "kg*m^2"),...
    'D', compParam('D', 0.05, 'AutoRename', true, 'Tunable', true, 'Unit', "m"),...
    'kV', compParam('kV', 900, 'AutoRename', true, 'Tunable', true, 'Unit', "RPM/V"),...
    'Rm', compParam('Rm',0.117, 'AutoRename', true, 'Tunable', true, 'Unit', "Ohm"));
%%
QROpt = QuadRotor('Frame', frame, 'Battery', batt, 'Propeller', prop, 'PMSMMotor', motor);
%%
save QROpt.mat QROpt