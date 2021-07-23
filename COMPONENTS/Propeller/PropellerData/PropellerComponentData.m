classdef PropellerComponentData < handle
    % Wrapper for props in ComponentData that allow for filtering by
    % product line
    properties (Constant)
        AllProductLines = ("E" | "EP" | "F" | "MR" | "SF" | "R")
    end
    properties
        ProductLines = ("E" | "EP" | "F" | "MR" | "SF" | "R")
        UnwantedChars = ("B4" | "EP" | "RH" | "LH" | "WCAR" | "F1-GT")
        AllCD ComponentData
    end
    properties (Access = private)
        SKUs
    end
    properties (Dependent)
        FilteredCD ComponentData
    end
    
    methods
        function obj = PropellerComponentData()
            addpath('APC_DATA')
            PropellerComponentData = ComponentData.importFromJSON('PropellerComponentDatabase.json');
            obj.AllCD = PropellerComponentData;
            obj.SKUs = vertcat(obj.AllCD.SKU);
        end
        
        function cd = get.FilteredCD(obj)
            I = contains(obj.SKUs, obj.ProductLines) & ~contains(obj.SKUs, obj.UnwantedChars);
            cd = obj.AllCD(I);
        end
        
        function save(obj)
            PropellerComponentData = obj;
            save PropellerComponentData.mat PropellerComponentData
        end
    end
end

