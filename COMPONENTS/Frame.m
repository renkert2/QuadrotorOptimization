classdef Frame < SystemElement
    %FRAME Summary of this class goes here
    %   Detailed explanation goes here
    properties
        Mass extrinsicProp = extrinsicProp("Mass", NaN, 'Unit', 'kg')
        Price extrinsicProp = extrinsicProp("Price", NaN, 'Unit', "USD")
        J_f compParam = compParam('J_f',NaN(3), 'Unit', 'kg*m^2', 'Assumptions', "real", 'Description', 'Inertia Tensor of Frame about its COM') % Momement of inertia of frame and static components about COM
        d compParam = compParam('d',NaN, 'Unit', 'm', 'Description', 'Distance from center to rotor')
    end
    
    properties (SetAccess = private)
        l compParam
    end
    
    methods
        function obj = Frame(varargin)
            obj = obj@SystemElement(varargin{:});
            obj.init_super();
        end
        
        function DefineParams(obj)
            obj.l = compParam('l',NaN, 'Unit', 'm', 'Description', 'Perpendicular distance from rotor to axes', 'Dependent', true);
            setDependency(obj.l,@(d) sqrt(2)/2*d, obj.d);
            obj.l.update();
            
            DefineParams@SystemElement(obj);
        end
    end
end

