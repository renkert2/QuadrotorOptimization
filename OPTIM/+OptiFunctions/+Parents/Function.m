classdef Function < handle & matlab.mixin.Heterogeneous
    %OPTIOBJECTIVE Summary of this class goes here
    %   Detailed explanation goes here
    properties
        Description string
        Sym string
        Unit string
    end
    
    properties
        LB
        UB
        Enabled logical = true
    end
    
    methods (Abstract)
        Value(obj)
    end
    
    % Methods to be Overriden by Subclasses
    methods 
        function f = val2f(obj, v)
            f = v;
        end
        
        function v = f2val(obj, f)
            v = f;
        end
        
        function g = val2g(obj, v)
            g = v;
        end
        
        function v = g2val(obj, g)
            v = g;
        end
    end
    
    methods (Sealed)   
        function e = getEnabled(obj_array)
            obj_array = obj_array(:);
            E = vertcat(obj_array.Enabled);
            e = obj_array(E);
        end
        
        function g = g(obj_array, varargin)
            g = arrayfun(@(obj) val2g(obj, obj.Value(varargin{:})), obj_array);
        end
        
        function f = f(obj_array, varargin)
            f = arrayfun(@(obj) val2f(obj, obj.Value(varargin{:})), obj_array);
        end
        
        function v = v(obj_array, varargin)
            v = arrayfun(@(obj) obj.Value(varargin{:}), obj_array);
        end
    end
end
