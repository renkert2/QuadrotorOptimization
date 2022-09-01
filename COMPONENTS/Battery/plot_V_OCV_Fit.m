load Lipo_42V_Lookup.mat LiPo_42V_Lookup
b = Battery();

%%
%%
fig = figure(1);
plot(LiPo_42V_Lookup.SOC, LiPo_42V_Lookup.V_OCV, 'LineWidth', 2)
hold on
fplot(b.V_OCV_curve*b.V_OCV_nominal, [0 1], 'LineWidth', 2)
hold off

xlabel("$$q$$")
ylabel("$$v_{OCV}(q)$$")

legend(["Experimental Data", "Polynomial Fit"])

formatFigures(fig, 'Width',5,'Height',3,'LineWidth', 2)