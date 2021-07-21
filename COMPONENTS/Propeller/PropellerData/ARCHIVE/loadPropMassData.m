table_path = "C:\Users\prenk\Box\ARG_Research\PropellerData\APC_Technical_Data.xlsx";
opts = detectImportOptions(table_path);
svars = ["Price___", "Weight", "Diameter_INCHES_", "Pitch_INCHES_"];
opts.SelectedVariableNames = svars;
opts = setvaropts(opts, svars, 'TrimNonNumeric',true,'Type','double');
% opts = setvartype(opts, svars, 'double');

oz2kg = 0.0283495;
in2m = 0.0254;

massTable = readtable(table_path,opts);
massTable.Properties.VariableNames = ["Price", "Weight_kg", "Diameter_m", "Pitch_m"];
massTable.Weight_kg = massTable.Weight_kg*oz2kg;
massTable.Diameter_m = massTable.Diameter_m*in2m;
massTable.Pitch_m = massTable.Pitch_m*in2m;
massTable = rmmissing(massTable)

%%
opts = fitoptions('poly2','Lower', [-Inf, 0, 0], 'Upper', [inf, 0, 0]);
massFit = fit(massTable.Diameter_m, massTable.Weight_kg, 'poly2', opts);

scatter(massTable.Diameter_m, massTable.Weight_kg)
hold on
plot(massFit)
hold off
title("Mass vs. Diameter")
xlabel("Diameter (m)")
ylabel("Weight (kg)")

%%
PF_Mass = propMassFit();


PF_Mass.Data = massTable;
PF_Mass.Fit = massFit;
PF_Mass.lb = min(massTable.Diameter_m);
PF_Mass.ub = max(massTable.Diameter_m);

save PF_Mass.mat PF_Mass