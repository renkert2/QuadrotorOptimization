classdef QRS_Function < OptiFunctions.Parents.Function
    properties
        QRS QuadRotorSystem
    end
    
    methods
        function obj = QRS_Function(qrs, varargin)
            obj = obj@OptiFunctions.Parents.Function(varargin{:});
            obj.QRS = qrs;
        end
    end
end