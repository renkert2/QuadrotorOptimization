file = fileread('StaticAeroData.json');
json = jsondecode(file);

%%
% Get 12x3.8SF Prop
prop = json.x12x38SF_dat;

u_k_P = 0.041429;
u_k_T = 0.11571;

t_k_P = 0.048;
t_k_T = 0.0968;

u_RPM = 390.5590*(u.rad/u.s)/u.rpm;
t_RPM = 427.0143*(u.rad/u.s)/u.rpm;

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
