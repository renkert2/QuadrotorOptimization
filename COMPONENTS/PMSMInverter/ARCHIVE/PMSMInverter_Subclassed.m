classdef PMSMInverter_Subclassed < DCDCConverter
    %DCDCCONVERTER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        rated_current double = 20; %Rated Current - Amps
        nominal_voltage double = 14.8; %Nominal (average) voltage - Volts
        eta = 0.98; % Efficiency
    end
    
    
    methods
        function r = setResistance(obj, rated_current, nominal_voltage, eta)
            if nargin == 4
                obj.eta = eta;
                obj.rated_current = rated_current;
                obj.nominal_voltage = nominal_voltage;
            end
            
            r = ((1-obj.eta)*obj.nominal_voltage)/obj.rated_current;
            obj.R_1 = r;
        end
        
        function obj = PMSMInverter_Subclassed(varargin)
             obj = obj@DCDCConverter(varargin{:}); 
             desc = ["DC Inductance", "DC Capacitance", "Virtual Inductance", "q Capacitance"];
             state_verts = obj.graph.Vertices(1:4);
             
             for i=1:length(state_verts)
                 state_verts(i).Description = desc(i);
             end
             
             obj.graph.Edges(3).Coefficient = sqrt(3/2);
             obj.graph.init();
        end
        
        function init(obj)
            obj.setResistance();
        end
    end
end

