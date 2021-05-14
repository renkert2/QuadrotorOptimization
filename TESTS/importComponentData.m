data = readcell('BatteryDatabase.xlsx', 'TextType', 'string', 'MissingRule', 'fill');

% Breakpoints: Component ; empty ; Parameter Data; empty ; Component Data
first_col = data(:,1);
[ir_comp,ic_comp] = find(cellfun(@(x) matchText(x,"Component"), data));
[ir_param, ic_param] = find(cellfun(@(x) matchText(x,"Parameter"), data));
[ir_data, ic_data] = find(cellfun(@(x) matchText(x,"Data"), data));

% Read Component
component = data{ir_comp,ic_comp+1};

%% Read Parameters
param_props = [first_col{ir_param:ir_data-1}];
param_data = data(ir_param:ir_data-1,2:end);
missing_cols = all(cellfun(@ismissing, param_data),1);
param_data = param_data(:,~missing_cols)';
param_table = cell2table(param_data, 'VariableNames', param_props);
%%
% Component Properties
comp_props = rmmissing([data{ir_data+1,ic:end}]);

% Get Data
varnames = [comp_props param_table{:,1}'];
data_table = cell2table(data(ir_data+2:end,:),'VariableNames',varnames);


function l = matchText(str,patt)
    if isa(str, 'string') || isa(str, 'char')
        l = contains(str,patt, 'IgnoreCase', true);
    else
        l = false;
    end
end