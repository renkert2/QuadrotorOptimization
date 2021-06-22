bm = BodyModel(QR_S500.Params);
lm = getLinearModel(bm);

sim_h = makeSimulinkModel(bm, 'BodyModel_Simulink');
open_system(sim_h)