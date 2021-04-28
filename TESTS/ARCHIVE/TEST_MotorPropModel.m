%load MotorPropSystem

motor_prop_model.StateNames
motor_prop_model.DisturbanceNames

disturbances = [11.1;0;0];

[t,x] = Simulate(motor_prop_model, [], disturbances, [0 10], 'Solver', @ode23tb);