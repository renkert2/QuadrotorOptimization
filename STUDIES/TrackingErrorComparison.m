[~, t, norm_err_init, ~] = trackingError(qrs_init);
[~, t_opt, norm_err_opt, ~] = trackingError(qrs_opt);
NPad = numel(norm_err_init) - numel(norm_err_opt);

delta_t = median(diff(t_opt));

t_opt = [t_opt; t_opt(end) + delta_t; t(end)];
norm_err_opt = [norm_err_opt; 0;0];
norm_err_opt = interp1(t_opt, norm_err_opt, t);

norm_err_delta = norm_err_opt - norm_err_init;

plot(t,norm_err_delta)

%%
title("Difference in Norm Error")
xlabel("Time (s)", 'Interpreter','latex')
ylabel("$$\Delta_e = e_{opt} - e_{init}$$", 'Interpreter','latex')
