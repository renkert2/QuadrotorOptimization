hb_mod = HolyBroS500();
modifyAeroCoeffs(hb_mod);

%% Select Components To Serve as Initial Design Point
batt_CD = hb_mod.PT.Battery.Surrogate.CD;
batt_table = table(batt_CD);
batt_I = (batt_table.N_s == 4) & (batt_table.Q == 4000);
batt = batt_CD(batt_I);
%%
motor_CD = hb_mod.PT.Motor.Surrogate.CD;
motor_table = table(motor_CD);
motor_I = (motor_table.kV == 965) & (motor_table.Rm == 0.102);
motor = motor_CD(motor_I);
%%
prop_CD = hb_mod.PT.Propeller.Surrogate.CD.FilteredCD;
prop_table = table(prop_CD);
prop_I = (prop_table.D == 0.2286) & (prop_table.P == 0.1143);
prop = prop_CD(prop_I);

%%
comps = [batt, motor, prop];
loadValues(hb_mod.Params, comps);
hb_mod.update();



function modifyAeroCoeffs(hb)
hb.PT.Propeller.k_P_mod.Value = 1.25;
hb.PT.Propeller.k_T_mod.Value = 0.85;
end