batt = Battery('N_p', symParam('N_p_batt',1));
%%
qr = QuadRotor('Battery', batt);
qr.Name = "Quadrotor";
%%
np_vals = linspace(0.999,1,50);
flight_times_nosim = zeros(size(np_vals));
flight_times_sim = flight_times_nosim;
flight_times_cupdate = flight_times_nosim;
qr.updateSymParamVals();
qr.calcControllerGains;

for i = 1:numel(np_vals)
    qr.updateSymParamVals(np_vals(i));
    flight_times_nosim(i) = flightTime(qr,'SimulationBased',false);
    flight_times_sim(i) = flightTime(qr,'SimulationBased',true, 'InterpolateTime', true);
    disp(qr.K_PID_height)
end

for i = 1:numel(np_vals)
    qr.updateSymParamVals(np_vals(i));
    qr.calcControllerGains;
    flight_times_cupdate(i) = flightTime(qr,'SimulationBased',true);
    disp(qr.K_PID_height)
end

%%
figure(1)
plot(np_vals, flight_times_nosim)
title("Static Objective")
xlabel('N_p')
ylabel('t')

figure(2)
plot(np_vals, flight_times_sim, np_vals, flight_times_cupdate)
title("Simulation-Based Objective")
xlabel('N_p')
ylabel('t')
legend(["Static Control Gains", "Updated Control Gains"])

%%
np_vals = linspace(0.9999999999,1,50);