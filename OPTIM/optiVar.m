classdef optiVar < handle
    %OPTIVAR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Child compParam

        x0 double
        lb double
        ub double
        
        Enabled logical = true
        scaleFactor double
        Scaled logical = true
    end
    
    properties (Dependent)
        Value
    end
    
    properties (Hidden)
        EnabledDefault logical = true
    end
    
    methods
        function obj = optiVar(child, lb, ub, opts)
            arguments
                child compParam
                lb double = -Inf
                ub double = Inf
                opts.Enabled = true
                opts.Scaled = true
                opts.ScaleFactor double = []
            end
            
            obj.Child = child;
            obj.lb = lb;
            obj.ub = ub;
 
            obj.Enabled = opts.Enabled;
            obj.EnabledDefault = opts.Enabled;
            
            if ~isempty(opts.ScaleFactor)
                obj.scaleFactor = opts.ScaleFactor;
            end            
        end
        
        function set.Child(obj,child)
            obj.Child = child;
            assert(child.Dependent == false, "Child Param of Optimization Variable must be Independent");
            obj.x0 = child.Value;
        end
        
        function set.Value(obj, value)
            obj.Child.Value = value;
        end
        
        function val = get.Value(obj)
            val = obj.Child.Value;
        end
        
        function i = isEnabled(obj)
            i = vertcat(obj.Enabled);
        end      
        
        function x0 = X0(obj)
            x0 = scale(obj, filterEnabled(obj, 'x0'));
        end
        
        function lb = LB(obj)
            lb = scale(obj, filterEnabled(obj, 'lb'));
        end
        
        function ub = UB(obj)
            ub = scale(obj, filterEnabled(obj, 'ub'));
        end
        
        function xall = XAll(obj, x)
            if nargin == 2
                xall = vertcat(obj.Value);
                xall(isEnabled(obj)) = unscale(obj,x);
            elseif nargin == 1
                xall = vertcat(obj.Value);
            end
        end
        
        function setEnabled(obj, arg, bool)
            arguments
                obj
                arg
                bool logical
            end
            
            assert(numel(arg) == numel(bool), 'Length of arg and bool args must be the same');
            if isa(arg, "double") || isa(arg, "logical")
                optivars = obj(arg);
            elseif isa(arg,"string")
                optivars = obj(find(obj, arg));
            else
                error('Invalid Argument');
            end
            
            for i = 1:numel(arg)
                optivars(i).Enabled = bool(i);
            end
        end
        
        function setVals(obj, val)
            val = unscale(obj, val);
            j = find(isEnabled(obj));
            for i = 1:numel(val)
                obj(j(i)).Value = val(i);
            end
        end
        
        function reset(obj_array)
            for i = 1:numel(obj_array)
                obj = obj_array(i);
                obj.Value = obj.x0;
                obj.Enabled = obj.EnabledDefault;
            end
        end
        
        function [pcs,pc] = percentChange(obj, val)
            if nargin == 1
                val = vertcat(obj.Value);
            end
            
            x0 = vertcat(obj.x0);
            pc = (val-x0)./x0;
            pcs = compose("%0.2f %%", 100*pc);
        end
        
        function sf = get.scaleFactor(obj)
            if isempty(obj.scaleFactor)
                sf = obj.x0;
            else
                sf = obj.scaleFactor;
            end
        end
        
        function o = get(obj, syms)
            childs = [obj.Child];
            child_syms = [childs.Sym];
            i = ismember(child_syms, syms);
            o = obj(i);
        end
        
        function s = latex(obj_array, varargin)
            s = latex(vertcat(obj_array.Child), varargin{:});
        end
        
        function tbl = dispTable(obj_array, child_fields, optim_fields, opts)
            arguments
                obj_array
                child_fields string = ["Sym", "Value", "Unit", "Description", "Parent"]
                optim_fields string = ["x0", "lb", "ub", "percentChange", "Enabled", "Scaled", "scaleFactor"]
                opts.ChildOpts cell = {}
            end
            % Modifies method from Mixin.Custom Display
            childs = vertcat(obj_array.Child);
            tbl = dispTable(childs, child_fields, opts.ChildOpts{:});
            for i = 1:numel(optim_fields)
                field = optim_fields(i);
                switch field
                    case "percentChange"
                        val = percentChange(obj_array);
                    otherwise
                        val = vertcat(obj_array.(field));
                end
                tbl.(optim_fields(i)) = val;
            end
            if nargout == 0
                disp(tbl);
            end
        end
        
        function t = dispTableLatex(obj)
            t = dispTable(obj, ["Sym", "Value", "Unit"], ["x0", "lb", "ub", "percentChange"], 'ChildOpts',{'LatexSym', true, 'LatexOpts',{'UnitFlag',false,'InlineArg',"$"}});
            t.Properties.VariableNames = ["Variable", "$X^*$", "Unit", "$X_0$", "LB", "UB", "\% Change"];
        end
    end
    
    methods (Hidden)
        function X = filterEnabled(obj, prop)
            vec = vertcat(obj.(prop));
            X = vec(isEnabled(obj));
        end
        
        function s = scaleFactors(obj)
            s = filterEnabled(obj, 'scaleFactor');
            i = vertcat(obj.Scaled);
            s(~i) = 1;
        end
        
        function x_s = scale(obj, x)
            if nargin > 1
                x_s = x./scaleFactors(obj);
            elseif nargin == 1
                x_s = filterEnabled(obj, 'Value')./scaleFactors(obj);
            end
        end
        
        function x_u = unscale(obj,x)
            if nargin > 1
                x = x(:); % Assert x is a column vector
                x_u = x.*scaleFactors(obj);
            elseif nargin == 1
                x_u = filterEnabled(obj, 'Value');
            end
        end
        
        function v = linspace(obj, n)
            v = linspace(obj.lb, obj.ub, n);
        end
         
        function s = parentTypes(obj_array) 
            s = string.empty(numel(obj_array), 0);
            for i = 1:numel(obj_array)
                s(i) = class(obj_array(i).Parent);
            end
            s = reshape(s,size(obj_array));
        end
    end
    
    methods (Hidden)
        
    end
end

