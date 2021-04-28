classdef CompliantMotorProp < System
    methods
        function obj = CompliantMotorProp(opts)
            arguments
                opts.pmsmmotor = PMSMMotor('Name', "Motor");
                opts.shaftcoupler = ShaftCoupler('Name', "Shaft");
                opts.propeller = Propeller('Name', "Propeller");
            end

            components = [opts.pmsmmotor, opts.shaftcoupler, opts.propeller];
            
            ConnectP = {[opts.pmsmmotor.Ports(2), opts.shaftcoupler.Ports(1)];
                [opts.shaftcoupler.Ports(2), opts.propeller.Ports(1)]};
            
            obj = obj@System(components, ConnectP);
        end
    end
end