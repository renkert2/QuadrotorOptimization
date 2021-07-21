classdef SingleRotor < System
    methods
        function obj = SingleRotor(opts)
            arguments
                opts.battery Battery = Battery('Name', "Battery");
                opts.dcbus DCBus_CurrentEquivalence = DCBus_CurrentEquivalence('Name', 'Bus', 'N_inputs',1,'N_outputs',1);
                opts.pmsminverter PMSMInverter = PMSMInverter('Name', "Inverter");
                opts.pmsmmotor PMSMMotor = PMSMMotor('Name', "Motor");
                opts.shaftcoupler ShaftCoupler = ShaftCoupler('Name', "Shaft");
                opts.propeller Propeller = Propeller('Name', "Propeller");
            end
            
            components = [opts.battery, opts.dcbus, opts.pmsminverter, opts.pmsmmotor, opts.shaftcoupler, opts.propeller];
            
            ConnectP = {[opts.battery.Ports(1) opts.dcbus.Ports(1)];...
                [opts.pmsminverter.Ports(1), opts.dcbus.Ports(2)];...
                [opts.pmsminverter.Ports(2), opts.pmsmmotor.Ports(1)];...
                [opts.pmsmmotor.Ports(2), opts.shaftcoupler.Ports(1)];...
                [opts.shaftcoupler.Ports(2), opts.propeller.Ports(1)];...
                };
            
            obj = obj@System(components, ConnectP);  
        end
    end
end

