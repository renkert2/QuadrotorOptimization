o.Objective = OptiFunctions.FlightTime();

range = 1000:-25:300;
warning off
[oo,range] = EpsilonConstraint(o,OptiFunctions.Price, "UB", range);

%%
I_valid = [oo.exitflag]>0;
oo = oo(I_valid);
range = range(I_valid);

%%
plot(range, [oo.F_opt]);
xlabel("$$p$$ (\$)", 'Interpreter', 'latex');
ylabel("Flight Time (s)", 'Interpreter', 'latex');
title("Multi-Objective: Flight Time vs. Price");

