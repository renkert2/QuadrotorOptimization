%% Init
close all
addpath(genpath('COMPONENTS'));

%% Instantiate Components - Add Symbolic Parameter for Propeller Diameter
pmsmmotor = PMSMMotor('Name', "Motor", 'J', symParam('J',1e-3));
shaftcoupler = ShaftCoupler('Name', "Shaft");
propeller = Propeller('Name', "Propeller", 'D', symParam('PropD',0.1270), 'J', symParam('J', 4.4e-05));

%% Connect Components

components = [pmsmmotor, shaftcoupler, propeller];

ConnectP = {[pmsmmotor.Ports(2), shaftcoupler.Ports(1)];
[shaftcoupler.Ports(2), propeller.Ports(1)]};

   
g_sys = Combine(components, ConnectP);
motor_prop_model = GraphModel(g_sys);