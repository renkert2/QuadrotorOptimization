cd C:\Users\prenk\Box\ARG_Research\QuadrotorOptimization\
addpath(genpath('COMPONENTS'))

motor = PMSMMotor();
coupler = ShaftCoupler();
prop = Propeller();

comps = [motor, coupler, prop];

ConnectP = {[motor.Ports(2) coupler.Ports(1)];
    [coupler.Ports(2) prop.Ports(1)]};

g_sys = Combine(comps, ConnectP);

g_sys.plot()

model_sys = GraphModel(g_sys);