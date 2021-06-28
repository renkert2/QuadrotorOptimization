classdef PowerTrain_SimpleModel < Model
    %POWERTRAIN_SIMPLEMODEL Modifies PowerTrain Model with simplifying
    %assumptions
    methods
        function obj = PowerTrain_SimpleModel(pt_model)
            obj = obj@Model();
            
            % STATES
            %dyn_eqns dot(x) = dyn_eqns(x)
            %1.         x1       "Battery"        "Battery SOC"           Abstract
            %2.         x2       "Motor"        "Inductance (i_q)"      Current
            %3.         x3       "Motor"        "Inertia (omega_m)"     AngularVelocity
            
            states = [1,2,3];
            obj.StateDescriptions = ["Battery SOC"; "Motor Current"; "Rotor Speed"];
            obj.Nx = numel(states);
            
            % INPUTS
            obj.Nu = 1;
            obj.InputDescriptions = ["Inverter Input"];
            
            
            % OUTPUTS:
            %1-3.  States 1-3
            %4.  Bus Voltage
            %5.  Bus Current
            %6.  Inverter Current (DC)
            %7.  Inverter Voltage (Q)
            %8. Torque
            %9. Thrust - Total
            
            outs = [states 10 11 12 13 21 20];
            obj.Ny = numel(outs);
            obj.OutputDescriptions = [obj.StateDescriptions; "Bus Voltage"; "Bus Current"; "Inverter Current (DC)"; "Inverter Voltage (q)"; "Torque"; "Total Thrust"];
            
            obj.SymVars = SymVars('Nx', obj.Nx, 'Nd', obj.Nd, 'Nu', obj.Nu); % We need this in order to set f_sym and g_sym
            
            digits 8 % Set VPA Digits to 8
            
            % Set f_sym
            dyn_eqns = vpa(pt_model.f_sym(states));
            obj.f_sym = combine(vpa(obj.convertSyms(dyn_eqns))); % combine makes sym expression a bit simpler
            
            % Set g_sym
            g_sym_mod = obj.convertSyms(vpa(pt_model.g_sym(outs)));
            g_sym_mod(end) = g_sym_mod(end)*4; % Convert Thrust per Propeller to Total Thrust
            obj.g_sym = combine(g_sym_mod); % combine makes sym expression a bit simpler
            
            obj.Params = pt_model.Params;
            
            obj.init();
        end
        
        function lm = getLinearModel(obj) % Overrides default method in Model
            u_mod = obj.SymVars.u;
            
            % Cut Battery Dynamics out of Dynamic Model
            x_mod = obj.SymVars.x(2:end);
            f_sym_mod = obj.f_sym(2:end);
            g_sym_mod = obj.g_sym([3,end]);
            
            A = jacobian(f_sym_mod, x_mod);
            B = jacobian(f_sym_mod, u_mod);
            
            C = jacobian(g_sym_mod, x_mod);
            D = jacobian(g_sym_mod, u_mod);
            
            lm = LinearModel();
            lm.Nx = 2;
            lm.Nu = 1;
            lm.Nd = 1; % Treat battery SOC as disturbance
            lm.Ny = 2;
            lm.StateDescriptions = obj.StateDescriptions(2:end);
            lm.InputDescriptions = obj.InputDescriptions;
            lm.DisturbanceDescriptions = "Battery SOC";
            lm.OutputDescriptions = obj.OutputDescriptions([3,end]);
            sv = obj.SymVars;
            sv.x = sv.x(2:end);
            sv.d = sym('x1');
            lm.SymVars = sv;
            p = [obj.Params];
            lm.Params = p;
            
            lm.A_sym = A;
            lm.B_sym = B;
            lm.E_sym = zeros(lm.Nx,1);
            lm.C_sym = C;
            lm.D_sym = D;
            lm.H_sym = zeros(lm.Ny,1);
            
            lm.f0_sym = zeros(lm.Nx,1);
            lm.g0_sym = zeros(lm.Ny,1);
            
            lm.init();
        end
    end
    
    %% Helper Functions
    methods (Access = private)
        function syms_out = convertSyms(obj, syms_in)
            syms_bal = convertToBalanced(obj,syms_in);
            syms_out = convertStates(obj,syms_bal);
        end
        
        function sym_bal = convertToBalanced(~, sym_unbal)
            % Replaces independent rotor variables with the same variable so that each
            % rotor has the same symbolic variable
            persistent from to
            if isempty(from) || isempty(to)
                from = {};
                to = {};
                
                from{1} = [sym('u2'), sym('u3'), sym('u4')];
                to{1} = repmat(sym('u1'),1,3);
                
                from{2} = [sym('x4'), sym('x6'), sym('x8')];
                to{2} = repmat(sym('x2'),1,3);
                
                from{3} = [sym('x5'), sym('x7'), sym('x9')];
                to{3} = repmat(sym('x3'),1,3);
            end
            
            sym_bal = subs(sym_unbal,[from{:}], [to{:}]);
        end
        
        function sym_out = convertStates(obj, sym_in)
            from = [sym('x1'); sym('x2'); sym('x3')];
            to = obj.SymVars.x;
            sym_out = subs(sym_in, from,to);
        end
    end
end

