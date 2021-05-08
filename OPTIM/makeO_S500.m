load QR_S500.mat QR_S500

o = Optimization(QR_S500);
o.initializeFromQR;

o.optiVars(1).ub = 0.35;

O_S500 = o;

save O_S500.mat O_S500;