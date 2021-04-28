classdef Battery < Component
    % Battery Cell Specifications from: Nemes et. al. "Parameters identification using experimental measurements for equivalent circuit Lithium-Ion cell models"
    % Battery Pack specifications (N_s and N_p) from: Ferry, "Quadcopter Plant Model and Control System Development With MATLAB/Simulink Implementation"
    % Pack specs from Turnigy Graphene Panther 4000mAh 3S 75C Battery Pack w/XT90
    
    % All parameters specified per cell except for N_series and N_parallel
    
    properties
        N_p {mustBeParam} = compParam('N_p',1) % Number of cells in parallel
        N_s {mustBeParam} = compParam('N_s',3) % Number of cells in series
        Q {mustBeParam} = 14400 % Coulombs
        R_s {mustBeParam} = (10e-3) / 3 % Series Resistance - Ohms - From Turnigy Website
        
        specificEnergy {mustBeParam} =  0.412 / (14400*11.1) % kg / J
        
        variableV_OCV logical = true
        V_OCV_nominal {mustBeParam} = 3.7 %Nominal Open Circuit Voltage = V_OCV_nominal*V_OCV_curve(q)
        V_OCV_curve = symfun(1, sym('q')) % Protected in set method
    end
        
    properties (Dependent)
        Energy % Joules
        Capacity % Coulombs = 1 A*s
        V_OCV_pack
    end
    
    properties (SetAccess = private)
        V_OCV_averaged
        
        Averaged_SOC double = 1 % SOC at which V_OCV(q) = V_OCV_Average
        Nominal_SOC double = 1 % SOC at which V_OCV(q) = V_OCV_nominal
    end
    
    methods
        function E = get.Energy(obj)
            E = obj.N_s*obj.N_p*obj.Q*obj.V_OCV_nominal;
        end
        
        function C = get.Capacity(obj)
            C = obj.Q*obj.N_p;
        end
        
        function V = get.V_OCV_pack(obj)
           V = obj.N_s*obj.V_OCV_nominal*obj.V_OCV_curve;
        end
        
        function setV_OCV_curve(obj,arg)
            if isa(arg, 'BattLookup')
                vocv = obj.fitV_OCV(arg.SOC, arg.V_OCV, arg.V_OCV_nominal);
                obj.V_OCV_curve = vocv;
            elseif isa(arg, 'symfun')
                obj.V_OCV_curve = arg;
            else
                error('V_OCV_curve must be of type BattLookup or symfun');
            end
             
            nq = double(vpasolve(obj.V_OCV_curve == 1));
            obj.Nominal_SOC = nq(1);
            
            ave_vocv_curve = double(vpaintegral(obj.V_OCV_curve, 0, 1));
            aq = double(vpasolve(obj.V_OCV_curve == ave_vocv_curve));
            obj.Averaged_SOC = aq(1);
            
            obj.V_OCV_averaged = ave_vocv_curve*obj.V_OCV_nominal;
        end
    end
    
    methods
        function init(obj)
            if obj.variableV_OCV
                if isequal(obj.V_OCV_curve, symfun(1, sym('q')))
                    load Lipo_42V_Lookup.mat LiPo_42V_Lookup
                    setV_OCV_curve(obj,LiPo_42V_Lookup);
                end
            else
                obj.V_OCV_curve = symfun(1, sym('q'));
                obj.V_OCV_averaged = obj.V_OCV_nominal;
                
                obj.Averaged_SOC = 1; % SOC at which V_OCV(q) = V_OCV_Average
                obj.Nominal_SOC = 1;
            end
            
            mp = extrinsicProp("Mass", obj.Energy*obj.specificEnergy);
            mp.Parent = obj;
            obj.Params(end+1,1) = mp;
        end
    end
    
    methods (Access = protected)
        function DefineComponent(obj)
            % Capacitance Types
            C(1) = Type_Capacitance(obj.V_OCV_curve(sym('x'))); % Capacitance Type for Q*V_OCV
            C(2) = Type_Capacitance("x");
            
            % PowerFlow Types
            P(1) = Type_PowerFlow(obj.V_OCV_curve(sym('xt'))*sym('xh'));
            P(2) = Type_PowerFlow("xt^2");
            
            % Vertices
            Vertex(1) = GraphVertex_Internal('Description', "Battery SOC", 'Capacitance', C(1), 'Coefficient', obj.N_s*obj.N_p*obj.Q*obj.V_OCV_nominal, 'Initial', 1, 'VertexType','Abstract');
            Vertex(2) = GraphVertex_External('Description', "Load Current", 'VertexType', 'Current');
            Vertex(3) = GraphVertex_External('Description', "Heat Sink", 'VertexType', 'Temperature');
            
            % Inputs
            
            % Edges
            Edge(1) = GraphEdge_Internal('PowerFlow',P(1),'Coefficient',obj.N_s*obj.V_OCV_nominal,'TailVertex',Vertex(1),'HeadVertex',Vertex(2));
            Edge(2) = GraphEdge_Internal('PowerFlow',P(2),'Coefficient',obj.N_s/obj.N_p*obj.R_s,'TailVertex',Vertex(2),'HeadVertex',Vertex(3));
            
            g = Graph(Vertex, Edge);
            obj.Graph = g;

            % Ports
            p(1) = ComponentPort('Description','Load Current','Element', Vertex(2));
            obj.Ports = p;
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
        
        function mah = CoulombsTomAh(c)
            mah = 0.2778*c;
        end
    end
end

