%% init
o.Objective = OptiFunctions.TrackingError(qrs);
o.setDefaultConstraints();

%% Battery Optimization
setEnabled(o.OptiVars, 3:6, false(1,4))