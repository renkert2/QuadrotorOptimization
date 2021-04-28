
p.Battery = Battery('Name', "Battery", 'variableV_OCV', false);
p.DCBus = DCBus_CurrentEquivalence('Name', 'Bus', 'N_inputs',1,'N_outputs',4);
p.PMSMInverter = PMSMInverter('Name', "Inverter", 'R_1',0, 'R_2',0);

replicated_comps = Replicate([p.PMSMInverter], 4);
pmsminverters = replicated_comps{1};
%%

components = [p.Battery, p.DCBus, replicated_comps{:}];

ConnectP = {[p.Battery.Ports(1) p.DCBus.Ports(1)]};

for i = 1:4
    ConnectP{end+1,1} = [pmsminverters(i).Ports(1),p.DCBus.Ports(1+i)];
end

sys = System(components, ConnectP);

%%
mod = sys.Model;
x = mod.SymVars.x;
u = mod.SymVars.u;
d = mod.SymVars.d;

syms v_q v_dc i_q i_dc v_b i_b

subs_array = ...
[d(1) 0;...
 d(2) 0;...
 d(3) i_q;...
 d(4) 0;...
 d(5) i_q;...
 d(6) 0;...
 d(7) i_q;...
 d(8) 0;...
 d(9) i_q;...
 d(10) 0;...
 u(1) u(1);...
 u(2) u(1);...
 u(3) u(1);...
 u(4) u(1)
];

g_sym_mod = subs(mod.g_sym,subs_array(:,1), subs_array(:,2));
%%
v_b = g_sym_mod(2);
i_b = g_sym_mod(3);
i_dc = g_sym_mod(4);
v_q = g_sym_mod(5);

in_power = v_b * i_b
out_power = 4*v_q*i_q

eta = simplifyFraction(out_power / in_power)

% 


    
