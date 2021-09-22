classdef QR_Function < OptiFunctions.Parents.Function
    properties
        QR QuadRotor
    end
    
    methods
        function obj = QR_Function(qr)
            obj.QR = qr;
        end
    end
end