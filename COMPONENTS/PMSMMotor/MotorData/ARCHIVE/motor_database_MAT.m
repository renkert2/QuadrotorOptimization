fileName = 'motor_database.json';
raw = fileread(fileName);
motor_data = jsondecode(raw);
specs_fields = ["Type", "Weight", "Diameter", "Length", "Shaft_Diameter", "Maximum_Amps", "Maximum_RPM"];
const_fields = ["kV", "Rm", "Io"];
for i = 1:numel(motor_data)
    for f = specs_fields
        if isfield(motor_data(i).SPECS, f)
            motor_data(i).SPECS.(f) = processUnit(motor_data(i).SPECS.(f));
        else
            motor_data(i).SPECS.(f) = NaN;
        end
    end
    
    for c = const_fields
        if isfield(motor_data(i).CONSTANTS, c)
            num = motor_data(i).CONSTANTS.(c);
            if isa(num,'struct')
                val = num.Value;
                
                if val==0 || isempty(val)
                    val = NaN;
                end
                motor_data(i).CONSTANTS.(c) = val; 
            elseif isnumeric(num) && num > 0
                motor_data(i).CONSTANTS.(c) = num;
            else
                motor_data(i).CONSTANTS.(c) = NaN;
            end
        else
            motor_data(i).CONSTANTS.(c) = NaN;
        end
    end
end

save('motor_data.mat','motor_data');

function val = processUnit(val_struct)
    persistent u
    if isempty(u)
        u = symunit;
    end
    
    if isa(val_struct, 'struct')
        val = val_struct.Value;
        unit = val_struct.Unit;
        if ~isempty(unit)
            val = double(separateUnits(unitConvert(val*u.(unit),'SI')));
        end
    else
        val = val_struct;
    end
end

