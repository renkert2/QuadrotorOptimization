table_path = "C:\Users\prenk\Box\ARG_Research\PropellerData\propDataTable.csv";
propTable = readtable(table_path, 'TextType', 'string', 'ReadVariableNames', true);
propTable.Properties.VariableNames = ["Name", "Diameter", "Pitch", "Path"];
%%
propStruct = table2struct(propTable);
for i = 1:numel(propStruct)
    path = propStruct(i).Path;
    data = readtable(path);
    propStruct(i).meanCt = mean(table2array(data(:,2)));
    propStruct(i).meanCp = mean(table2array(data(:,3)));
    propStruct(i).Static.Data = data;
end

%%
save('propStruct.mat', 'propStruct')

%%
load('propStruct.mat','propStruct')
%% Lookup Vals
ps_filtered = propStruct([propStruct.Diameter] < 1 & [propStruct.Pitch] < 1);
diameter = [ps_filtered.Diameter]';
pitch = [ps_filtered.Pitch]';
ct = [ps_filtered.meanCt]';
cp = [ps_filtered.meanCp]';

%%
ct_interp = scatteredInterpolant(diameter,pitch,ct, 'natural');
ct_fit = fit([diameter pitch], ct, 'Poly33');

cp_interp = scatteredInterpolant(diameter,pitch,cp, 'natural');
cp_fit = fit([diameter pitch], cp, 'Poly33');



%% Create Valid Boundary
% bound = boundary(diameter, pitch, 0);
% 
% test_point = [0 .25 10; 0 .1 10];
% [d_min, x_d_min, y_d_min] = p_poly_dist(test_point(1,:), test_point(2,:), diameter(bound), pitch(bound), true);
% 
% dist_func = @(D,P) p_poly_dist(D, P, diameter(bound), pitch(bound), true);

bound = Boundary([diameter pitch]);
plot(bound)
%%


%%
% propFits.Interp.ct = ct_interp;
% propFits.Interp.cp = cp_interp;
% propFits.Fit.ct = ct_fit;
% propFits.Fit.cp = cp_fit;
% propFits.Boundary.points = [diameter(bound) pitch(bound)];
% propFits.Boundary.distance_func = dist_func;
% 
% save propFits.mat propFits

PF_Aero = propAeroFit();
Data.Diameter = diameter;
Data.Pitch = pitch;
Data.ct = ct;
Data.cp = cp;
PF_Aero.Data = Data;
Interp.ct = ct_interp;
Interp.cp = cp_interp;
PF_Aero.Interp = Interp;
Fit.ct = ct_fit;
Fit.cp = cp_fit;
PF_Aero.Fit = Fit;
PF_Aero.Boundary = bound;
PF_Aero.PropStruct = propStruct;

save PF_Aero.mat PF_Aero

%% 

figure(1)
plot(PF_Aero.Fit.ct, [PF_Aero.Data.Diameter, PF_Aero.Data.Pitch], PF_Aero.Data.ct)
zlim([0 max(PF_Aero.Data.ct)])
title('Propeller Thrust Coefficient')
xlabel('Diameter (m)')
ylabel('Pitch (m)')
zlabel('C_T')


figure(2)
plot(PF_Aero.Fit.cp, [PF_Aero.Data.Diameter, PF_Aero.Data.Pitch], PF_Aero.Data.cp)
zlim([0 max(PF_Aero.Data.cp)])
title('Propeller Power Coefficient')
xlabel('Diameter (m)')
ylabel('Pitch (m)')
zlabel('C_P')
