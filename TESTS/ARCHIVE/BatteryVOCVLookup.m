batt = Battery();
load LiPo_42V_Lookup.mat LiPo_42V_Lookup
%%
figure
plot(LiPo_42V_Lookup.SOC, LiPo_42V_Lookup.V_OCV)
hold on
fplot(batt.V_OCV_curve*batt.V_OCV_nominal, [0 1])
hold off

eqn = "$V_{OCV}(q) \approx "+strrep(string(vpa(batt.V_OCV_curve(sym('q')),3)),"*","")+"$";

legend(["Lookup Table", eqn],'Interpreter', 'latex')
title("Polynial Approximation of V_{OCV}(q)")
ylabel('$V_{OCV}$', 'Interpreter', 'latex')
xlabel('$SOC$','Interpreter', 'latex')