f = @(x).1*x.^2-500;
p = @(x)5*x+100;

x0 = 50;
f0 = f(x0);
p0 = p(x0);
del_p = @(x) p(x) - p0;
del_f = @(x) f(x) - f0;

fplot(del_p, [0 100]);
hold on
fplot(del_f, [0 100]);
fplot(@(x) del_f(x)./del_p(x), [0 100]);
hold off