inv = PMSMInverter('R_1', symParam('R_1', 0.01), 'R_2', symParam('R_2', 0), 'InverterType', 'ConstantLoss');

qr = QuadRotor('PMSMInverter',inv);
%%

u_vals = linspace(0,1,10);
eta = zeros(size(u_vals));

qr.updateSymParamVals([0.01;0])

for i = 1:numel(u_vals)
    eta_all = calcEfficiency(qr,u_vals(i),1);
    eta(i) = eta_all.Inverter;
end

plot(u_vals,eta)

%% 
batt = Battery('N_s', symParam('N_s',3), 'N_p', symParam('N_p', 1));
qr = QuadRotor('Battery', batt, 'PMSMInverter',inv);

%%
np_vals = linspace(0.1,2,25);
ns_vals = linspace(0.1,12,25);

[X,Y] = meshgrid(np_vals, ns_vals);
ft = nan(size(X));
power = nan(size(X));
eta_batt = nan(size(X));
eta_inv  = nan(size(X));
eta_mot = nan(size(X));
eta_sys = nan(size(X));

for i = 1:numel(np_vals)
    for j = 1:numel(ns_vals)
        try
            sp_vals = [np_vals(i); ns_vals(j);.01;0];
            qr.updateSymParamVals(sp_vals);
            power(j,i) = qr.SS_QAve.y(5)*qr.SS_QAve.y(4);
            ft(j,i) = flightTime(qr);
            eta_all = calcEfficiency(qr);
            eta_batt(j,i) = eta_all.Battery;
            eta_inv(j,i) = eta_all.Inverter;
            eta_mot(j,i) = eta_all.Motor;
            eta_sys(j,i) = eta_all.Sys;
        end
    end
end
%%
figure(1)
surf(X,Y,eta_sys)
%zlim([0.992 0.993])
xlabel('N_p')
ylabel('N_s')
title('Powertrain Efficiency at Steady State')

%%
figure(2)
surf(X,Y,ft)
xlabel('N_p')
ylabel('N_s')
title('Flight Time')

%%
figure(3)
surf(X,Y,power)
xlabel('N_p')
%%

np_vals = linspace(0.1,2,25);
ns_vals = 3./np_vals;

ft = nan(size(np_vals));
power = nan(size(np_vals));
eta_batt = nan(size(np_vals));
eta_inv  = nan(size(np_vals));
eta_mot = nan(size(np_vals));
eta_sys = nan(size(np_vals));

for i = 1:numel(np_vals)
    try
        sp_vals = [np_vals(i); ns_vals(i);.01;0];
        qr.updateSymParamVals(sp_vals);
        power(i) = qr.SS_QAve.y(5)*qr.SS_QAve.y(4);
        ft(i) = flightTime(qr);
        eta_all = calcEfficiency(qr);
        eta_batt(i) = eta_all.Battery;
        eta_inv(i) = eta_all.Inverter;
        eta_mot(i) = eta_all.Motor;
        eta_sys(i) = eta_all.Sys;
    end
end

%%
figure(1)
plot(np_vals,eta_mot)
%zlim([0.992 0.993])
xlabel('N_p')
ylabel('N_s')

%%
figure(2)
plot(np_vals,ft)
xlabel('N_p')
ylabel('N_s')

%%
figure(3)
plot(np_vals,power)
xlabel('N_p')