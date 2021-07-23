addpath('APC_DATA')
PropellerComponentData = ComponentData.importFromJSON('PropellerComponentDatabase.json');

skus = vertcat(PropellerComponentData.SKU);
I = validSKU(skus);

PropellerComponentData = PropellerComponentData(I);

save PropellerComponentData.mat PropellerComponentData

function i = validSKU(sku)
    wanted_chars = ("E" | "EP" | "F" | "MR" | "SF" | "R");
    unwanted_chars = ("B4" | "EP" | "RH" | "LH" | "WCAR" | "F1-GT");

    i = contains(sku, wanted_chars) & ~contains(sku, unwanted_chars);
end