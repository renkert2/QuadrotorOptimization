classdef PMSMInverter < Component
    properties
        InverterType InverterTypes = InverterTypes.VoltageDependent
        
        I_max {mustBeParam} = 20; %Rated Current - Amps
%         nominal_voltage {mustBeParam} = 14.8; %Nominal (average) voltage - Volts
%         eta {mustBeParam} = 0.85; % Efficiency
        
        L {mustBeParam} = 0;
        C {mustBeParam} = 0;
        R_1 {mustBeParam} = 0.01;
        R_2 {mustBeParam} = 0;
        
        Mass {mustBeParam} = extrinsicProp('Mass', 0.03, 'Unit', "kg");
    end
    
    methods
        function r = setResistance(obj, I_max, nominal_voltage, eta)
            % Placeholder - would like to calculate R_1 and R_2 given rated values and efficiency here
        end
    end
    
    methods
        function init(obj)
            obj.setResistance();
        end
    end
    
    methods (Access = protected)
        function DefineComponent(obj)
            % Capacitance Types
            C(1) = Type_Capacitance("x");
            
            % PowerFlow Types
            P(1) = Type_PowerFlow("xt*xh");
            P(2) = Type_PowerFlow("u1*xt*xh");
            P(3) = Type_PowerFlow("xt^2");

            % Internal Vertices
            desc = ["DC Inductance", "q Capacitance"];
            type = [VertexTypes.Current, VertexTypes.Voltage];
            cap = C(1);
            coeff = [1*obj.L, 1*obj.C];
            init = 0;
            for i = 1:2
                V(i) = GraphVertex_Internal('Description',desc(i),'Capacitance',cap,'Coefficient',coeff(i),'Initial',init, 'VertexType', type(i));
            end
            
            % External Vertices
            desc = ["Input Voltage (DC)", "Output Current (q)", "Heat Sink"];
            type = [VertexTypes.Voltage, VertexTypes.Current, VertexTypes.Temperature];
            init = [0 0 0];
            for i = 1:3
                V(i+2) = GraphVertex_External('Description',desc(i), 'VertexType', type(i));
            end
            
            % Inputs
            I(1) = GraphInput("d");
            
            % Edges
            E(1) = GraphEdge_Internal('PowerFlow',P(2),'Coefficient',1,'TailVertex',V(3),'HeadVertex',V(1), 'Input', I(1));
            E(2) = GraphEdge_Internal('PowerFlow',P(1),'Coefficient',sqrt(2/3),'TailVertex',V(1),'HeadVertex',V(2));
            E(3) = GraphEdge_Internal('PowerFlow',P(1),'Coefficient',1,'TailVertex',V(2),'HeadVertex',V(4));
            
            if obj.InverterType == InverterTypes.ConstantLoss
                P(4) = Type_PowerFlow("xt");
                E(4) = GraphEdge_Internal('PowerFlow',[P(3) P(4)],'Coefficient',[1*obj.R_1 1*obj.R_2],'TailVertex',V(1),'HeadVertex',V(5));
            elseif obj.InverterType == InverterTypes.VoltageDependent
                P(4) = Type_PowerFlow("u1*xt");
                E(4) = GraphEdge_Internal('PowerFlow',[P(3) P(4)],'Coefficient',[1*obj.R_1 1*obj.R_2],'TailVertex',V(1),'HeadVertex',V(5), 'Input', I(1));
            elseif obj.InverterType == InverterTypes.CurrentDependent
                P(4) = Type_PowerFlow("(1/u1)*xt^2");
                E(4) = GraphEdge_Internal('PowerFlow',[P(3) P(4)],'Coefficient',[1*obj.R_1 1*obj.R_2],'TailVertex',V(1),'HeadVertex',V(5), 'Input', I(1)); 
            else
                error('Invalid InverterType')
            end
            
            g = Graph(V, E);
            obj.Graph = g;
            
            p(1) = ComponentPort('Description',"Voltage Input (DC)",'Element',obj.Graph.Edges(1));
            p(2) = ComponentPort('Description',"Current Output (q)",'Element',obj.Graph.Edges(3));
            p(3) = ComponentPort('Description',"Heat Sink",'Element',obj.Graph.Vertices(5));
            obj.Ports = p;
        end
    end
end

