[omega] = getTS(qrs.SimOut,'omega_out').Data(:,1);
%%
bm = qrs.QR.BM;
kt = bm.K_T.Value;
kq = bm.K_Q.Value;
l = bm.l.Value;

omega_range = [min(omega), qr.SS_QAve.RotorSpeed, max(omega)];

force_range = kt*omega_range.^2;
torque_range = kq*omega_range.^2;
