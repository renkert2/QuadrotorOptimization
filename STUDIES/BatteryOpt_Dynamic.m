%% Optimize Battery Only
o.OptiVars.setEnabled([1 2 5 6], false(1,4));
% Switch into Continuous Mode
setDependent(o.DependentParams, true);
o.QRS.RefTrajVar = "TimeSeries";
%% Set Trajectory
o.QRS.RefTraj = ReferenceTrajectory3D("Lemniscate");
o.QRS.setLQR(0.1)

%% Set to Smallest Feasible Battery and Evaluate Performance
o.OptiVars(3).Value = 6;
o.OptiVars(4).Value = 1000;
qr.Frame.Mass.Value = 0.5788;
o.updateQR(false);

qrso_B1 = qrs.SimOut;

%% Set to Largest Feasible Battery and Evaluate Performance
o.OptiVars(3).Value = 6;
o.OptiVars(4).Value = 6000;
qr.Frame.Mass.Value = 5*0.5788;
o.updateQR(false);

qrso_B2 = qrs.SimOut();

%% Compare Results
%plotTrajectory([qrso_B1, qrso_B2], qr.BM, 'LegendNames',["Small Battery", "Large Battery"])
plotTrackingError([qrso_B1, qrso_B2], "LegendNames", ["Small Battery", "Large Battery"])