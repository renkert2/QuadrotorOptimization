classdef Battery < Component
    % Battery Cell Specifications from: Nemes et. al. "Parameters identification using experimental measurements for equivalent circuit Lithium-Ion cell models"
    % Battery Pack specifications (Q,N_s,N_p) from: Ferry, "Quadcopter Plant Model and Control System Development With MATLAB/Simulink Implementation"
    
    % All parameters specified per cell except for N_series and N_parallel
    
    properties
        Q {mustBeParam} = 2520  %Cell Capacity - Coulombs
        
        C_1 {mustBeParam} = 690.349 % Farads
        R_1 {mustBeParam} = 0.003 % Ohms
        C_2 {mustBeParam} = 1700.084 % Farads
        R_2 {mustBeParam} = 0.0606 % Ohms
        R_s {mustBeParam} = 0.09825 % Series Resistance - Ohms
        
        variableV_OCV logical = true
        V_OCV_nominal {mustBeParam} = 3.7 %Nominal Open Circuit Voltage = V_OCV_nominal*V_OCV_poly(q)
        V_OCV_curve = symfun(1, sym('q')) % Protected in set method
        
        N_s {mustBeParam} = 3 % Number of cells in series
        N_p {mustBeParam} = 1 % Number of cells in parallel
        
        massPerCell {mustBeParam} = 0.08/3 % kg
    end
    
    methods
        function set.V_OCV_curve(obj,arg)
            if isa(arg, 'BattLookup')
                vocv = obj.fitV_OCV(arg.SOC, arg.V_OCV, arg.V_OCV_nominal);
                obj.V_OCV_curve = vocv;
            elseif isa(arg, 'symfun')
                obj.V_OCV_curve = arg;
            else
                error('V_OCV_curve must be of type BattLookup or symfun');
            end
        end
    end
    
    methods (Access = protected)
        function init(obj)
           if obj.variableV_OCV
               if isequal(obj.V_OCV_curve, symfun(1, sym('q')))
                   load Lipo_42V_Lookup.mat LiPo_42V_Lookup
                   obj.V_OCV_curve = LiPo_42V_Lookup;
               end
           end
        end
        
        function DefineComponent(obj)
            % Capacitance Types
            C(1) = Type_Capacitance(obj.V_OCV_curve(sym('x'))); % Capacitance Type for Q*V_OCV
            C(2) = Type_Capacitance("x");
            
            % PowerFlow Types
            P(1) = Type_PowerFlow(obj.V_OCV_curve(sym('xt'))*sym('xh'));
            P(2) = Type_PowerFlow("xt*xh");
            P(3) = Type_PowerFlow("xt^2");
            
            % Vertices
            Vertex(1) = GraphVertex_Internal('Description', "Battery SOC", 'Capacitance', C(1), 'Coefficient', obj.N_s*obj.N_p*obj.Q*obj.V_OCV_nominal, 'Initial', 1, 'VertexType','Abstract');
            Vertex(2) = GraphVertex_Internal('Description', "Capacitance 1", 'Capacitance', C(2), 'Coefficient', obj.N_p/obj.N_s*obj.C_1, 'Initial', 0, 'VertexType', 'Voltage');
            Vertex(3) = GraphVertex_Internal('Description', "Capacitance 2", 'Capacitance', C(2), 'Coefficient', obj.N_p/obj.N_s*obj.C_2, 'Initial', 0, 'VertexType', 'Voltage');
            Vertex(4) = GraphVertex_External('Description', "Load Current", 'VertexType', 'Current');
            Vertex(5) = GraphVertex_External('Description', "Heat Sink", 'VertexType', 'Temperature');
            
            % Inputs
            
            % Edges
            Edge(1) = GraphEdge_Internal('PowerFlow',P(1),'Coefficient',obj.N_s*obj.V_OCV_nominal,'TailVertex',Vertex(1),'HeadVertex',Vertex(4));
            Edge(2) = GraphEdge_Internal('PowerFlow',P(3),'Coefficient',obj.N_p/obj.N_s*1/obj.R_1,'TailVertex',Vertex(2),'HeadVertex',Vertex(5));
            Edge(3) = GraphEdge_Internal('PowerFlow',P(3),'Coefficient',obj.N_p/obj.N_s*1/obj.R_2,'TailVertex',Vertex(3),'HeadVertex',Vertex(5));
            Edge(4) = GraphEdge_Internal('PowerFlow',P(2),'Coefficient',1,'TailVertex',Vertex(4),'HeadVertex',Vertex(2));
            Edge(5) = GraphEdge_Internal('PowerFlow',P(2),'Coefficient',1,'TailVertex',Vertex(4),'HeadVertex',Vertex(3));
            Edge(6) = GraphEdge_Internal('PowerFlow',P(3),'Coefficient',obj.N_s/obj.N_p*obj.R_s,'TailVertex',Vertex(4),'HeadVertex',Vertex(5));
            
            g = Graph(Vertex, Edge);
            obj.Graph = g;

            % Ports
            p(1) = ComponentPort('Description','Load Current','Element', Vertex(4));
            obj.Ports = p;
            
            % extrinsicProps
            obj.extrinsicProps = extrinsicProp("Mass", obj.N_s*obj.N_p*obj.massPerCell);
        end
    end
    
    methods (Static)
        function V_OCV_sym = fitV_OCV(q, v, v_nominal, opts)
            arguments
                q double
                v double
                v_nominal double
                opts.poly_order double = 8
                opts.sym_var sym = sym('q');
            end
            
            v_normalized = v./v_nominal;
            
            digits_old = digits();
            digits(4);
            p_coeffs = vpa(polyfit(q,v_normalized,opts.poly_order));
            
            V_OCV_sym = symfun(poly2sym(p_coeffs, opts.sym_var), opts.sym_var);
            
            digits(digits_old);
        end
    end
end

