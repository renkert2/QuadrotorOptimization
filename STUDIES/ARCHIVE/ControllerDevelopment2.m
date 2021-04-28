batt = Battery('variableV_OCV', false);
qr = QuadRotor('Battery', batt);

%% Simulate with Incorrect u_bar
qr.Simulate('FeedForwardU', false)
%%
qr.Simulate('FeedForwardU', false,'FeedForwardT', true, 'GainMultiplier',500)

%% Figure Out IMP
[Am,Bm,Cm,Dm] = calcBodyModel(qr);
% Cm = eye(2);
% Dm = zeros(2,1);
b_ss = ss(Am,Bm,Cm,Dm);

[Apt, Bpt, Cpt, Dpt] = calcLinearMatrices(qr);
pt_ss = ss(Apt,Bpt,Cpt,Dpt);
gain = Dpt-Cpt*(Apt\Bpt);
dom_pole = min(abs(eigs(Apt)));
pt_simple = tf(gain, [(1/dom_pole), 1]);

%%
lg = series(pt_ss,b_ss);
lg_sym = tf2sym(tf(lg),sym('s'));
C = sym('Kp')+sym('s')*sym('Kd');
lg_sym = simplifyFraction(-C*lg_sym);

E_sym = simplifyFraction(1/(1+lg_sym));

fvt_e = subs(sym('s')*E_sym, sym('s'),0)

%% 
u_sched = 0:0.05:1;
T = zeros(size(u_sched));

for i = 1:numel(u_sched)
    T(i) = calcSteadyStateIO(qr, u_sched(i));
end

%%
plot(u_sched,T)
title("Thrust vs. Inverter Input")
xlabel('$u$','Interpreter', 'latex')
ylabel('$T$', 'Interpreter', 'latex')
