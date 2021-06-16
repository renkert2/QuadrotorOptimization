classdef Frame < SystemElement
    %FRAME Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Mass = extrinsicProp("Mass", 0.284 - 0.080 - 4*0.008 - 4*0.04)
        J_f (3,3) double = NaN % Momement of inertia of frame and static components about COM
        d double = NaN
    end
    
    properties (Dependent)
        l double
    end
    
    methods
        function obj = Frame(varargin)
            obj = obj@SystemElement(varargin{:});
            obj.init_super();
        end
        
        function l = get.l(obj)
            l = sqrt(2)/2*obj.d;
        end
    end
end

