% Goal: Investigate the noise caused by finite differencing.  
% Procedure: 1) vary the step size 2) for each step size, calculate the
% objective function multiple times to see if its consistent

step_range = 10.^linspace(-15, -1, 50);
fdiffs = struct();
for i = 1:numel(step_range)
    fdiffs(i).StepSize = step_range(i);
    [grad,grad_s] = fdiff(o, 'FiniteDifferenceStepSize', step_range(i));
    fdiffs(i).Grad = grad;
    fdiffs(i).GradS = grad_s;
end
save("fdiffs.mat", "fdiffs");

%%
for i = 1:numel(fdiffs)
    grad = fdiffs(i).Grad(:,3);
    fdiffs(i).Grad = grad;
end

%%
grad_array = abs([fdiffs.Grad]);
N_start = 35;
plot(step_range(N_start:end), grad_array(:,N_start:end))
loglog(step_range, grad_array)

legend(latex(o.OptiVars), 'Interpreter', 'latex')
xlabel("Step Size ($$\epsilon$$)", 'Interpreter', 'latex')