classdef QR_InitialDesign < HolyBroS500
    % Modifies HolyBroS500 to include similar components from the Battery,
    % Propeller, and Motor databases.  It is used as the initial design
    % point for the optimization studies
    properties
        InitialConfiguration
    end
    methods
        %% HolyBro S500
        % https://shop.holybro.com/s500-v2-kitmotor2216-880kv-propeller1045_p1153.htm
        
        function obj = QR_InitialDesign()
            obj = obj@HolyBroS500();
            modifyAeroCoeffs(obj);
            
            %% Select Components To Serve as Initial Design Point
            batt_CD = obj.PT.Battery.Surrogate.CD;
            batt_table = table(batt_CD);
            batt_I = (batt_table.N_s == 4) & (batt_table.Q == 4000);
            batt = batt_CD(batt_I);
            %%
            motor_CD = obj.PT.Motor.Surrogate.CD;
            motor_table = table(motor_CD);
            motor_I = (motor_table.kV == 965) & (motor_table.Rm == 0.102);
            motor = motor_CD(motor_I);
            %%
            prop_CD = obj.PT.Propeller.Surrogate.CD.FilteredCD;
            prop_table = table(prop_CD);
            prop_I = (prop_table.D == 0.2286) & (prop_table.P == 0.1143);
            prop = prop_CD(prop_I);
            
            %%
            comps = [batt, motor, prop];
            obj.InitialConfiguration = comps;
            loadValues(obj.Params, comps);
            obj.update();

            function modifyAeroCoeffs(qr)
                % Modifies Aerodynamic Coefficient Modifiers to reflect
                % experimental results with large KDE propellers.  Sample
                % Size includes 1 propeller and 2 batteries, so it isn't
                % very accurate.  
                qr.PT.Propeller.k_P_mod.Value = 1.25;
                qr.PT.Propeller.k_T_mod.Value = 0.85;
            end
        end
    end
end