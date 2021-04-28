classdef PMSMInverter < Component
    %DCDCCONVERTER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        rated_current double = 20; %Rated Current - Amps
        nominal_voltage double = 14.8; %Nominal (average) voltage - Volts
        eta = 0.85; % Efficiency
        
        L_1 double = 1e-4;
        C_1 double = 0;
        L_2 double = 0;
        C_2 double = 1e-4;
        R_1 double = 1e-2;
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
    end
    
    methods (Access = protected)
        function init(obj)
            obj.setResistance();
        end
        function DefineComponent(obj)
            % Capacitance Types
            C(1) = Type_Capacitance("x");
            
            % PowerFlow Types
            P(1) = Type_PowerFlow("xt*xh");
            P(2) = Type_PowerFlow("u1*xt*xh");
            P(3) = Type_PowerFlow("xt^2");
            
            % Vertices

            % Internal Vertices
            desc = ["DC Inductance", "DC Capacitance", "Virtual Inductance", "q Capacitance"];
            type = [VertexTypes.Current, VertexTypes.Voltage, VertexTypes.Current, VertexTypes.Voltage];
            cap = C(1);
            coeff = [obj.L_1, obj.C_1, obj.L_2, obj.C_2];
            init = 0;
            for i = 1:4
                V(i) = GraphVertex_Internal('Description',desc(i),'Capacitance',cap,'Coefficient',coeff(i),'Initial',init, 'VertexType', type(i));
            end
            
            % External Vertices
            desc = ["Output Current", "Input Voltage", "Heat Sink"];
            type = [VertexTypes.Current, VertexTypes.Voltage, VertexTypes.Temperature];
            init = [0 0 0];
            for i = 1:3
                V(i+4) = GraphVertex_External('Description',desc(i),'Initial', init(i), 'VertexType', type(i));
            end
            
            % Inputs
            I(1) = GraphInput("d_1");
            
            % Edges
            E(6) = GraphEdge();
            pflows = P([1,1,2,1,1,3]);
            coeffs = [1 1 sqrt(3/2) 1 1 obj.R_1];
            Emat = V([6 1;1 2;2 3;3 4;4 5;1 7]);
            for i = 1:6
                E(i) = GraphEdge_Internal('PowerFlow',pflows(i),'Coefficient',coeffs(i),'TailVertex',Emat(i,1),'HeadVertex',Emat(i,2));
            end
            E(3).Input = I(1);
            
            g = Graph(V, E);
            obj.graph = g;
            
            p(1) = ComponentPort('Description',"Voltage Input",'Element',obj.graph.Edges(1));
            p(2) = ComponentPort('Description',"Current Output",'Element',obj.graph.Edges(5));
            p(3) = ComponentPort('Description',"Heat Sink",'Element',obj.graph.Vertices(7));
            obj.Ports = p;
        end
    end
end

