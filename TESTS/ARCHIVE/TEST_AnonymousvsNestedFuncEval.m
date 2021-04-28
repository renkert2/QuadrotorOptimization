
inputs = 1;
disturbances = zeros(5,1);

x = single_rotor_model.x_init;


odeFunc = processArgs(single_rotor_model.CalcG, inputs, disturbances)
odeFunc([0],x)

function xfunc = processArgs(func, inputs_arg, disturbances_arg)
input_function_flag = false;
disturbance_function_flag = false;

if input_function_flag && disturbance_function_flag
    arg2 = @(t) inputs_arg(t);
    arg3 = @(t) disturbances_arg(t);
elseif input_function_flag
    xfunc = @(t,x) arg1(x, inputs_arg(t), repmat(disturbances_arg,1,numel(t)));
elseif disturbance_function_flag
    xfunc = @(t,x) arg1(x, repmat(inputs_arg,1,numel(t)), disturbances_arg(t));
else
    xfunc = @(t,x) arg1(x, repmat(inputs_arg,1,numel(t)), repmat(disturbances_arg,1,numel(t)));
end

xfunc = @(t,x) func(x, arg2, arg3);
end