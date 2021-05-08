N_p = compParam('N_p', 1);
N_s = compParam('N_s', 3);

M_b = extrinsicProp('Mass', NaN);
M_b.setDependency(@(N_p, N_s) N_p*N_s, [N_p; N_s]);

M_f = extrinsicProp('Mass', 10);

M_q = Combine([M_b, M_f]);

params = [M_q;N_p;N_s;M_b;M_f];
