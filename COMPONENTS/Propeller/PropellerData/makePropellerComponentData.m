addpath('APC_DATA')
PropellerComponentData = ComponentData.importFromJSON('PropellerComponentDatabase.json');

% Remove Specific Propellers
bad_props = ["LP13045EP"; "LP13040"];

skus = vertcat(PropellerComponentData.SKU);
bad_I = false(size(PropellerComponentData));
for i = 1:numel(bad_props)
    bad_I = bad_I | (contains(skus, bad_props(i), 'IgnoreCase', true));
end
PropellerComponentData = PropellerComponentData(~bad_I);

save PropellerComponentData.mat PropellerComponentData

