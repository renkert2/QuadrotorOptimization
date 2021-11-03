m = 1.5392;
g = 9.81;

A = [0 1; 0 0];
B = [0; 1/m];
C = [1 0];
D = [0];
sys = ss(A,B,C,D);

rho = 1;
Q = diag([1,0.1,0.1]);
R = rho;

K_lqi = lqi(sys,Q,R);

%% Simulate System - Step Input
% Normal state vector
set_param('DoubleIntQuad/StateModSwitch','sw','1');
set_param('DoubleIntQuad/ReferenceSwitch','sw','0');
set_param('DoubleIntQuad/DisturbanceSwitch','sw','1');
sim_out_notrackstate = sim('DoubleIntQuad');

% Replace position state with error signal
set_param('DoubleIntQuad/StateModSwitch','sw','0');
set_param('DoubleIntQuad/ReferenceSwitch','sw','0');
set_param('DoubleIntQuad/DisturbanceSwitch','sw','1');
sim_out_trackstate = sim('DoubleIntQuad');

% Create Plots
plot(get(sim_out_notrackstate.yout, 'r').Values);
hold on
plot(get(sim_out_notrackstate.yout, 'y').Values)
plot(get(sim_out_trackstate.yout, 'y').Values)
hold off
legend(["Reference", "Unmodified State", "Modified State"])
title("Step Input")

%% Simulate System - Disturbance Input
% Normal state vector
set_param('DoubleIntQuad/StateModSwitch','sw','1');
set_param('DoubleIntQuad/ReferenceSwitch','sw','1');
set_param('DoubleIntQuad/DisturbanceSwitch','sw','0');
sim_out_notrackstate = sim('DoubleIntQuad');

% Replace position state with error signal
set_param('DoubleIntQuad/StateModSwitch','sw','0');
set_param('DoubleIntQuad/ReferenceSwitch','sw','1');
set_param('DoubleIntQuad/DisturbanceSwitch','sw','0');
sim_out_trackstate = sim('DoubleIntQuad');

% Create Plots
plot(get(sim_out_notrackstate.yout, 'r').Values);
hold on
plot(get(sim_out_notrackstate.yout, 'y').Values)
plot(get(sim_out_trackstate.yout, 'y').Values)
hold off
legend(["Reference", "Unmodified State", "Modified State"])
title("Step Disturbance")