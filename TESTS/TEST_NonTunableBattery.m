%%
batt = Battery('Name', 'Battery',...
    'Q', compParam('Q',4000,'Unit', 'mAh', 'AutoRename', true, 'Tunable', true),...
    'N_p', compParam('N_p',1,'Unit', 'unit', 'AutoRename', true, 'Tunable', true),...
    'N_s', compParam('N_s',4, 'Unit', 'unit', 'AutoRename', true, 'Tunable', true),... % 4000mAh, No Dynamics
    'R_s', compParam('R_s', .004, 'Unit', "Ohm", 'AutoRename', true, 'Tunable', true),... % Measured with Battery Charger, likely not very accurate.  R_s = N_p/N_s R_p
    'Mass', extrinsicProp('Mass', 0.47735, 'Unit',"kg", 'AutoRename', true, 'Tunable', true),...
    'Price', extrinsicProp('Price', 59.99, 'Unit', "USD", 'AutoRename', true, 'Tunable', false),...
    'V_OCV_nominal', compParam('V_OCV_nom', 3.7, 'Unit', 'V', 'AutoRename', true, 'Tunable', true),...
    'variableV_OCV', false);
qr_batt = QuadRotor('Battery', batt);

%%

qr = QR_InitialDesign();
%%
b = qr.PT.Battery();
b.variableV_OCV = false;
b.V_OCV_nominal.Tunable=true;
b.init_super
b.Graph.init()
%%
qr.PT.init_super();
qr.PT.SystemCombine();
qr.PT.Graph.init();
%%
qr.PT.Model = GraphModel.empty();
%qr.PT.init_post();
%%

% Rebuild the System with the new battery
qr.init()