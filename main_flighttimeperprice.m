%% Initialize Objects
qr = QR_InitialDesign(); % Create quadrotor object of initial design
o = Optimization(qr);
o.Objective = OptiFunctions.FlightTimePerPrice(qr);

%% Continuous Optimization
%oo = o.Optimize();
load('OO_Mixed_ContinuousSolution_04052022.mat')

%% Discrete Search (Hybrid Approach)
o.resetQR()
so = Search(o,oo, Inf, 500, "SortMode","Distance", 'Plot', true);
fmin = so.F_opt;