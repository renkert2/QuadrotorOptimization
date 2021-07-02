motor = PMSMMotor('Name', "PMSM Motor");
motor_model = GraphModel(motor);
%% 
motor_model.StateNames
motor_model.InputNames
motor_model.DisturbanceNames

input_voltage = 1*sqrt(3/2);
load_torque = 0;

%disturbances = @(t) [input_voltage*ones(size(t)); load_torque*heaviside(t-0.25); 0*ones(size(t))]; % Apply load torque at 0.25 seconds
disturbances = [input_voltage; load_torque; 0]; % Apply load torque at 0.25 seconds
%%
close all
figure

motor_model.Simulate([], disturbances, [], [0 10]);

%% Test Motor with Algebraic Current State
