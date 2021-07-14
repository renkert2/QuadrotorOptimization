load motor_data_kde
Data = motor_data_kde;

% Parameter Template
Sym = ["Price", "Mass","D","kV","Rm","Io", "I_max"]';
Unit = ["USD", "kg","m","rpm/V","Ohm","A","A"]';
Description = ["Price", "Mass", "Diameter", "Speed Constant", "Phase Resistance", "No Load Current", "Max Current"]';
Component = repmat("PMSMMotor",numel(Sym),1);
param_table = table(Sym, Unit, Description, Component);
data_template = compParamValue.importFromTable(param_table);

% Create ComponentData Array
N = numel(Data);
obj_array = ComponentData.empty(N,0);

for i = 1:N
    cd = ComponentData();
    cd.Component = "PMSMMotor";
    
    d = Data(i);
    cd.Make = d.MAKE;
    cd.Model = d.MODEL;
    
    cpvdata = data_template;
    cpvdata(1).Value = d.PRICE;
    cpvdata(2).Value = d.SPECS.Weight;
    cpvdata(3).Value = d.SPECS.Diameter;
    cpvdata(4).Value = d.CONSTANTS.kV;
    cpvdata(5).Value = d.CONSTANTS.Rm;
    cpvdata(6).Value = d.CONSTANTS.Io;
    cpvdata(7).Value = d.SPECS.Maximum_Amps;
    
    cd.Data = cpvdata;
    obj_array(i,1) = cd;
end

%%
MotorComponentData = obj_array;
save MotorComponentData.mat MotorComponentData;