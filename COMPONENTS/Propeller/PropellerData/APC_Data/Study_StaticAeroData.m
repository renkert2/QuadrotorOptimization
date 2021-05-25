file = fileread('StaticAeroData.json');
json = jsondecode(file);

ptbls = cell2mat(struct2cell(json));

%%
for i = 1:20
    x = ptbls(i).THRUST_LBF;
    y = ptbls(i).Cp;
    y2 = ptbls(i).Ct;
    figure(1)
    hold on
    plot(x,y)
    hold off
    title("RPM vs. Cp")
    figure(2)
    hold on
    plot(x,y2)
    hold off
    title("RPM vs. Ct")
end

%% 
for i = 1:20
    x = ptbls(i).RPM;
    y = ptbls(i).THRUST_LBF;
    y2 = ptbls(i).TORQUE_IN_LBF;
    
    figure(1)
    subplot(1,2,1)
    hold on
    plot(x,y)
    xlim([1000 10000])
    title("Thrust Speed Curve")
    xlabel("RPM")
    ylabel("Thrust (lbf)")
    hold off
    
    subplot(1,2,2)
    hold on
    plot(x,y2)
    xlim([1000 10000])
    title("Torque Speed Curve")
    xlabel("RPM")
    ylabel("Torque (in-lbf)")
    hold off
end

%% Second Derivatives
for i = 1:20
    x = diff(ptbls(i).THRUST_LBF,2)
    x2 = diff(ptbls(i).TORQUE_IN_LBF,2)
    
    figure(2)
    hold on
    plot(x)
    hold off
    
    figure(3)
    hold on
    plot(x2)
    hold off
end
%% Find range of values for each field
arrayfun(@(x) min(x.THRUST_LBF), ptbls)

%% Fit Torque and Speed Coeffs to Experimental Data
p = Propeller();
ft = fittype({'x^2'});
torque_fit = fit(t_exp.RotorSpeed,t_exp.Torque,ft);
thrust_fit = fit(t_exp.RotorSpeed,t_exp.Thrust,ft);

K_T = Propeller.convCoeffToRevPerS(thrust_fit.a); % lumped rev/s
K_Q = Propeller.convCoeffToRevPerS(torque_fit.a); % lumped rev/s

k_T = prop.lumpedToThrustCoeff(K_T);
k_Q = prop.lumpedToTorqueCoeff(K_Q);

k_P = k_Q * 2*pi;
prop.D.Tunable = true;

prop.k_P.Value = k_P;
prop.k_T.Value = k_T;