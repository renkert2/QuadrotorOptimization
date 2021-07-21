classdef DCBus_CurrentEquivalence < Component
    %DCBUS Component
    
    properties
        N_inputs double {mustBeInteger} = 1
        N_outputs double {mustBeInteger} = 1
        
        L {mustBeParam} = 0
        R compParam = compParam("R", 0, 'Unit', 'Ohm')
        C {mustBeParam} = 0
    end
    
    methods (Access = protected)
        function DefineComponent(obj)
            % Capacitance Types
            C(1) = Type_Capacitance('x');
            
            % PowerFlow Types
            PF(1) = Type_PowerFlow('xt*xh');
            PF(2) = Type_PowerFlow('xt^2');
            
            % Counters   
            V_cnt = [1 obj.N_inputs obj.N_outputs 1];
            V_cumsum = cumsum([0 V_cnt]);
            
            I_cumsum = cumsum([0, obj.N_inputs, obj.N_outputs]);
            
            E_cumsum = cumsum([0 obj.N_inputs obj.N_outputs obj.N_inputs]);
            
            % Define Vertices that don't vary with N_inputs or N_outputs
            V(V_cumsum(1)+1) = GraphVertex_Internal(...
                'Description', "Internal Voltage",... 
                'VertexType',"Voltage",...
                'Capacitance',C(1),...
                'Coefficient',obj.C,...
                'Initial',0);            
                        
            V(V_cumsum(4)+1) = GraphVertex_External(...
                'Description', "Heat Sink",...
                'VertexType',"Temperature");
            
            % Elements that vary with N_inputs
            for i = 1:obj.N_inputs
                V(V_cumsum(2)+i) = GraphVertex_Internal(...
                    'Description', sprintf("Internal Current %d", i),...
                    'VertexType',"Current",...
                    'Capacitance',C(1),...
                    'Coefficient',obj.L,...
                    'Initial',0);
                
                E(E_cumsum(1)+i) = GraphEdge_Internal(...
                    'PowerFlow',PF(1),...
                    'Coefficient',1,...
                    'TailVertex',V(V_cumsum(2)+i),...
                    'HeadVertex',V(V_cumsum(1)+1));
                
                E(E_cumsum(3)+i) = GraphEdge_Internal(...
                    'PowerFlow',PF(2),...
                    'Coefficient',pop(obj.R),...
                    'TailVertex',V(V_cumsum(2)+i),...
                    'HeadVertex',V(V_cumsum(4)+1));
                
                p(i) = ComponentPort('Description', sprintf("Current Equivalence %d",i),'Element', V(V_cumsum(2)+i));
            end
            
            % Elements that vary with N_ouptuts
            for i = 1:obj.N_outputs
                V(V_cumsum(3)+i) = GraphVertex_External(...
                    'Description', sprintf("Ouptut Current %d", i),...
                    'VertexType',"Current");
                
                E(E_cumsum(2)+i) = GraphEdge_Internal(...
                    'PowerFlow',PF(1),...
                    'Coefficient',1,...
                    'TailVertex',V(V_cumsum(1)+1),...
                    'HeadVertex',V(V_cumsum(3)+i));
                
                p(end+1) = ComponentPort('Description', sprintf("Current Output %d",i),'Element', E(E_cumsum(2)+i));
            end
            
            p(end+1) = ComponentPort('Description',"Heat Sink",'Element',V(V_cumsum(4)+1));
              
            g = Graph(V,E);
            obj.Graph = g;
            
            % Ports
            obj.Ports = p; 
        end
    end
end

