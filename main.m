%% OptiSetup
qr = QR_InitialDesign;
qrs = QuadRotorSystem(qr);
o = Optimization(qrs);
o.Objective = OptiFunctions.TrackingError(qrs);
o.setDefaultConstraints();