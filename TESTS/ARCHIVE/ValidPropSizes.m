dyn_eqns = vpa(po.quad_rotor.Model.f_sym([2,3,4,5,12,16]));
dyn_eqns = subs(dyn_eqns, [sym('u2'), sym('u3'), sym('u4'), sym('x6'), sym('x8'), sym('x10')],[sym('u1'), sym('u1'), sym('u1'), sym('x4'), sym('x4'), sym('x4')]);
prop_eqn = dyn_eqns(end);

% Okay - what was happening was k_P was going negative from the fit.  Lets try a lower bound on P in terms of D.
% Ended up drawing a boundary around the dataset and constraining D,P to that boundary
%%
% Compare Resulting Coefficients to DJI Mavic Air II
T = readtable("CommercialDrones.xlsx", 'TextType', 'string');
MavicAir = T(3,:);

u = symunit();

vpa(unitConvert([MavicAir.Prop_Diameter_m, MavicAir.Prop_Pitch_m]*u.m, u.in),4)

load propStruct;
propTable = struct2table(propStruct);
propTable = sortrows(propTable, "Diameter");
closestProp = propTable(propTable.Name == "magf_7x4", :);
disp(closestProp.Static.Data)

% Now, see if this is anywhere near what our fit is telling us:
[mavic_kp, mavic_kt] = po.calcPropCoeffs([MavicAir.Prop_Diameter_m; MavicAir.Prop_Pitch_m])
[closest_kp, closest_kt] = po.calcPropCoeffs([closestProp.Diameter; closestProp.Pitch])

% The coefficients predicted by the fit are a little higher than the actual values, but we're in the ballpark.

%%
X = [MavicAir.Prop_Diameter_m; MavicAir.Prop_Pitch_m];
X_bad = [0.057; 0.02];
[xbad_kp, xbad_kt] = po.calcPropCoeffs(X_bad);
[x_ss, input_sched, reqd_speed, dyn_eqns] = po.calcSteadyState(X_bad, po.reqd_thrust);

%%
[x_ss, input_sched, reqd_speed, dyn_eqns] = po.calcSteadyState(X, po.reqd_thrust);
[t, y, p_f, errFlag] = po.Simulate(X, repmat(input_sched,4,1));
figure(1)
plot(t,y(:,28))
figure(2)
plot(t,y(:,1))
po.flightTime(X)



