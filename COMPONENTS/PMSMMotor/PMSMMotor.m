classdef PMSMMotor < Component
    %PMSM Motor
    %   Default parameters need updated
    
    properties
        L {mustBeParam} = compParam('L', 1.17e-4, 'Unit', "H") % Inductance - H
        J {mustBeParam} = compParam('J', 6.5e-6,'Unit', "kg*m^2") % Mechanical rotational inertia - Modified to better reflect Ferry's simulation results
        kV {mustBeParam} = compParam('kV', 900,'Unit', "RPM/V") % Torque/Speed Coefficient - Nm/A = Vs/rad
        Rm {mustBeParam} = compParam('Rm',0.117, 'Unit', "Ohm") % Phase Resistance - Ohms
        B_v {mustBeParam} = 0 % Viscous Friction - N*m*s
        T_c {mustBeParam} = 0 % Coulomb Friction
        sigmoid_a_param {mustBeParam} = 10 % Parameter used to approximate sign function with sigmoid function sig(w) = 2/(1+Exp(-a*w))-1
        
        M {mustBeParam} = extrinsicProp('Mass',0.04, 'Unit', "kg");
        D {mustBeParam} = compParam('D', 0.05, 'Unit', "m");
    end
    
    properties (Dependent)
       K_t 
    end
    
    properties (SetAccess = private)
        Fit paramFit
    end
    
    methods (Static)
        function K_t = calcTorqueConstant(P,lambda_m)
            % P - Total number of poles - not pole pairs
            % lambda_m - Magnetic Flux Linkage
            K_t = (P/2)*lambda_m;
        end
        
        function K_t = kVToKt(kV)
            % Convert kV in rpm/V to torque constant Kt in N*m/A = V/(rad/s)
            kV_radps = kV*(2*pi)/60;
            K_t = 1./kV_radps;
        end
        
        function kV = KtTokV(Kt)
            kV_radps = 1./Kt;
            kV =kV_radps/((2*pi)/60);
        end
        
        function J = calcInertia(M,D)
            % Estimate mass of rotor as M/2;
            % R = D/2;
            % J = MR^2 (Hoop moment of inertia)
            
            J = (M/2).*(D/2).^2;
        end
    end
    
        
    methods
        function K_t = get.K_t(obj)
            K_t = obj.kVToKt(obj.kV);
        end
        
        function init(obj)
            load MotorFit.mat MotorFit;
            MotorFit.Inputs = [obj.kV, obj.Rm];
            MotorFit.Outputs = [obj.M, obj.D];
            MotorFit.setOutputDependency;
            obj.Fit = MotorFit;
            
            obj.J.setDependency(@PMSMMotor.calcInertia, [obj.M, obj.D]);
        end
    end
    
    methods (Access = protected)  
        function DefineComponent(obj)
            % Capacitance Types
            C(1) = Type_Capacitance('x');
            
            % PowerFlow Types
            PF(1) = Type_PowerFlow('xt*xh');
            PF(2) = Type_PowerFlow('xt^2');
            
            syms xt
            sig = 2/(1+exp(-1*obj.sigmoid_a_param.*xt))-1;
            PF(3) = Type_PowerFlow(sig*xt);
            
            % Vertices
            V(1) = GraphVertex_Internal('Description', "Inductance (i_q)", 'Capacitance', C(1), 'Coefficient', 1*obj.L, 'Initial', 0, 'VertexType', 'Current');
            V(2) = GraphVertex_Internal('Description', "Inertia (omega_m)", 'Capacitance', C(1), 'Coefficient', 1*obj.J, 'Initial', 0, 'VertexType', 'AngularVelocity');
            
            V(3) = GraphVertex_External('Description', "Input Voltage (v_q)", 'VertexType', 'Voltage');
            V(4) = GraphVertex_External('Description', "Mechanical Load (T_l)", 'VertexType', 'Torque');
            V(5) = GraphVertex_External('Description', "Heat Sink", 'VertexType', 'Temperature');
            
            % Inputs
            
            % Edges
            E(1) = GraphEdge_Internal(...
                'PowerFlow',PF(1),...
                'Coefficient',1,...
                'TailVertex',V(3),...
                'HeadVertex',V(1));
            
            E(2) = GraphEdge_Internal(...
                'PowerFlow',PF(1),...
                'Coefficient',sqrt(3/2)*obj.K_t,...
                'TailVertex',V(1),...
                'HeadVertex',V(2));
            
            E(3) = GraphEdge_Internal(...
                'PowerFlow',PF(1),...
                'Coefficient',1,...
                'TailVertex',V(2),...
                'HeadVertex',V(4));
            
            E(4) = GraphEdge_Internal(...
                'PowerFlow',PF(2),...
                'Coefficient',1*obj.Rm,...
                'TailVertex',V(1),...
                'HeadVertex',V(5));
            
            E(5) = GraphEdge_Internal(...
                'PowerFlow',[PF(2) PF(3)],...
                'Coefficient',[1*obj.B_v 1*obj.T_c],...
                'TailVertex',V(2),...
                'HeadVertex',V(5));
                       
            g = Graph(V, E);
            obj.Graph = g;
            
            % Ports            
            p(1) = ComponentPort('Description',"Voltage Input",'Element',E(1));
            p(2) = ComponentPort('Description',"Torque Output",'Element',E(3));
            p(3) = ComponentPort('Description',"Heat Sink",'Element',V(5));
            obj.Ports = p;
        end
    end
end

