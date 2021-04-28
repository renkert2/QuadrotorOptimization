qr = QuadRotor();
s = qr.SimpleModel;
lm = qr.LinearDynamicModel;

%% Body Model
[Am,Bm,Cm,Dm] = calcBodyModel(qr);
% Cm = eye(2);
% Dm = zeros(2,1);
b_ss = ss(Am,Bm,Cm,Dm);

%% PowerTrain Model
[Apt, Bpt, Cpt, Dpt] = calcLinearMatrices(qr);
pt_ss = ss(Apt,Bpt,Cpt,Dpt);
gain = Dpt-Cpt*(Apt\Bpt);
dom_pole = min(abs(eigs(Apt)));
den = [(1/dom_pole), 1];
pt_simple = [tf(gain(1), den); tf(gain(2), den)];

%% Inner Speed Loop
PI_speed = pidtune(pt_simple(1),'PI');
speed_loop = feedback(PI_speed*pt_simple(1),1);
step(speed_loop)

%%
rlocusplot(PI_speed*pt_simple(1))
title('Root Locus: Speed Control Loop')
hold on
poles = pole(speed_loop);
zeros = zero(speed_loop);
plot(poles, 'xr');
plot(zeros, 'or');
hold off


%% Outer Height Loop
outer_plant = series(speed_loop*gain(2)/gain(1), b_ss);
PID_height = pidtune(outer_plant,'PID');
height_loop = feedback(PID_height*outer_plant, 1);
step(height_loop)

%%
rlocusplot(PID_height*outer_plant)
title('Root Locus: Height Control Loop')
hold on
poles = pole(height_loop);
zeros = zero(height_loop);
plot(real(poles), imag(poles), 'xr');
plot(real(zeros), imag(zeros), 'or');
hold off
