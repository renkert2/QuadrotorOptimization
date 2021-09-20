%% init
load FlightLog_07052021.mat;
qr = HolyBroS500;

%loadValues(qr.Params, FL.Components);
qr.update();

%%
pmv = PowertrainModelValidation();
pmv.FL = FL;
pmv.PT = qr.PT;
pmv.init;
pmv.setTestConditions;
%%
%pmv.PT.Propeller.k_P.Value = 0.026; % Tuned value
pmv.PT.Params.update;
pmv.Simulate;
pmv.plotPairs;

%% 
mv = ModelValidation();
mv.FL = FL;
qrs = QuadRotorSystem(qr);
qrs.init();
mv.M = qrs;

mv.setPairs;
mv.setTestConditions();
%%
mv.M.QR.PT.Propeller.k_T.Value = 0.068;
mv.M.QR.update;
mv.Simulate
mv.plotPairs;

%%
smv = StaticModelValidation();
smv.QR = qr;
smv.FL = FL;
smv.init;
smv.setTestConditions;

%% 
file = fileread('StaticAeroData.json');
json = jsondecode(file);

%%
% Get 12x3.8SF Prop
file = fileread('StaticAeroData.json');
json = jsondecode(file); 
prop = json.x12x38SF_dat;

propdat = FL.Components(5).Data;
u_k_P = filterSym(propdat, "k_P").Value;
u_k_T = filterSym(propdat, "k_T").Value;

t_k_P = pmv.PT.Propeller.k_P.Value;
t_k_T = mv.M.QR.PT.Propeller.k_T.Value;

u_RPM = 428.6906*(u.rad/u.s)/u.rpm;
t_RPM = 450.0314*(u.rad/u.s)/u.rpm;

%%
x = prop.RPM;
y = prop.Cp;
y2 = prop.Ct;
t = tiledlayout(1,2);
title(t,"RPM vs. Aerodynamic Coefficients - 12x3.8SF");
nexttile(1)
plot(x,y, 'DisplayName', "APC Data")
yline(u_k_P, '-r', 'DisplayName', 'Untuned Value')
yline(t_k_P, '-g', 'DisplayName', 'Tuned Value')
xline(u_RPM, '-r', 'DisplayName', 'Untuned Rotor Speed')
xline(t_RPM, '-g', 'DisplayName', 'Tuned Rotor Speed')
xlabel("RPM");
ylabel("$$k_P$$",'Interpreter','latex');
legend

nexttile(2)
plot(x,y2, 'DisplayName', "APC Data")
yline(u_k_T, '-r', 'DisplayName', 'Untuned Value')
yline(t_k_T, '-g', 'DisplayName', 'Tuned Value')
xline(u_RPM, '-r', 'DisplayName', 'Untuned Rotor Speed')
xline(t_RPM, '-g', 'DisplayName', 'Tuned Rotor Speed')
xlabel("RPM");
ylabel("$$k_T$$",'Interpreter','latex');
legend

%%
