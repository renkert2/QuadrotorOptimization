classdef Battery < Component
    % Battery Cell Specifications from: Nemes et. al. "Parameters identification using experimental measurements for equivalent circuit Lithium-Ion cell models"
    % Battery Pack specifications (N_s and N_p) from: Ferry, "Quadcopter Plant Model and Control System Development With MATLAB/Simulink Implementation"
    % Pack specs from Turnigy Graphene Panther 4000mAh 3S 75C Battery Pack w/XT90
    
    % All parameters specified per cell except for N_series and N_parallel
    
    properties
        % Independent Params
        N_p compParam = compParam('N_p', 1, 'Unit', "unit") % Number of cells in parallel
        N_s compParam = compParam('N_s', 3, 'Unit', "unit") % Number of cells in series
        Q compParam = compParam('Q', 4000, 'Unit', "mAh") % mAh
    
        % Dependent Params - Dependency set in init()
        R_s compParam = compParam('R_s', (10e-3) / 3, 'Unit', "Ohm") % Series Resistance - Ohms - From Turnigy Website
        Mass compParam = extrinsicProp("Mass", NaN, 'Unit', "kg"); % Dependent param defined in init
        
        variableV_OCV logical = true
        V_OCV_nominal double = 3.7 %Nominal Open Circuit Voltage = V_OCV_nominal*V_OCV_curve(q)
        V_OCV_curve = symfun(1, sym('q')) % Protected in set method
    end
    
    properties (SetAccess = private)
        R_p compParam = compParam('R_p', NaN, 'Unit', "Ohm", 'Description', 'Pack Resistance') % Dependent compParam, pack resistance
        Capacity compParam = compParam('Capacity', NaN, 'Unit', "Amp*second", 'Description', 'Pack Capacity'); % Dependent compParam A*s
        V_OCV_pack function_handle % Depends on q
    end
    
    properties (SetAccess = private)
        V_OCV_averaged
        
        Averaged_SOC double = 1 % SOC at which V_OCV(q) = V_OCV_Average
        Nominal_SOC double = 1 % SOC at which V_OCV(q) = V_OCV_nominal
        
        Fit paramFit
    end
    
    methods        
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
            
            
            load BatteryFit.mat BatteryFit;
            BatteryFit.Inputs = [obj.N_s, obj.Q];
            BatteryFit.Outputs = [obj.R_s, obj.Mass];
            BatteryFit.setOutputDependency();
            obj.Fit = BatteryFit;
            
            rpfun = @(N_s,N_p,R_s) N_s./N_p.*R_s;
            setDependency(obj.R_p, rpfun, [obj.N_s, obj.N_p, obj.R_s]);
            obj.R_p.Dependent = true;
            
            capfun = @(N_p,Q) N_p.*Battery.mAhToCoulombs(Q);
            setDependency(obj.Capacity, capfun, [obj.N_p, obj.Q]);
            obj.Capacity.Dependent = true;
            
            V_pack_sym = obj.N_s.*obj.V_OCV_nominal.*obj.V_OCV_curve;
            obj.V_OCV_pack = matlabFunction([obj.N_s], V_pack_sym, {sym('q')});
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
            voltage_coeff = obj.N_s*obj.V_OCV_nominal;
            energy_coeff = voltage_coeff*pop(obj.Capacity);
            
            Vertex(1) = GraphVertex_Internal('Description', "Battery SOC", 'Capacitance', C(1), 'Coefficient', energy_coeff, 'Initial', 1, 'VertexType','Abstract');
            Vertex(2) = GraphVertex_External('Description', "Load Current", 'VertexType', 'Current');
            Vertex(3) = GraphVertex_External('Description', "Heat Sink", 'VertexType', 'Temperature');
            
            % Inputs
            
            % Edges
            Edge(1) = GraphEdge_Internal('PowerFlow',P(1),'Coefficient',voltage_coeff,'TailVertex',Vertex(1),'HeadVertex',Vertex(2));
            Edge(2) = GraphEdge_Internal('PowerFlow',P(2),'Coefficient',pop(obj.R_p),'TailVertex',Vertex(2),'HeadVertex',Vertex(3));
            
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
            mah = c/3.6;
        end
        
        function c = mAhToCoulombs(mAh)
            c = 3.6*mAh;
        end
    end
end

