% Construct paramFit object
pf = paramFit(2,3);

% Set Data
load PropellerComponentData.mat PropellerComponentData;
[t,~] = table(PropellerComponentData);

input_data = [t.D, t.P];
output_data = [t.k_P, t.k_T, t.Mass];
pf.setData(input_data, output_data);
pf.setBoundary();

%%
pf.cftool(3)


%% Set Models
% k_P
% ft_kp = fittype( 'poly33' );
% opts_kp = fitoptions( 'Method', 'LinearLeastSquares' );
% opts_kp.Robust = 'Bisquare';

ft_kp = fittype( 'loess' );
opts_kp = fitoptions( 'Method', 'LowessFit' );
opts_kp.Normalize = 'on';
opts_kp.Robust = 'Bisquare';
opts_kp.Span = 0.3;

% k_T
% ft_kt = fittype( 'poly33' );
% opts_kt = fitoptions( 'Method', 'LinearLeastSquares' );
% opts_kt.Robust = 'Bisquare';

ft_kt = fittype( 'loess' );
opts_kt = fitoptions( 'Method', 'LowessFit' );
opts_kt.Normalize = 'on';
opts_kt.Robust = 'Bisquare';
opts_kt.Span = 0.3;

% MassFit
% ft_mass = fittype( 'poly22' );
% opts_mass = fitoptions( 'Method', 'LinearLeastSquares' );
% opts_mass.Robust = 'Bisquare';

ft_mass = fittype( 'loess' );
opts_mass = fitoptions( 'Method', 'LowessFit' );
opts_mass.Normalize = 'on';
opts_mass.Robust = 'Bisquare';
opts_mass.Span = 0.3;

pf.setModels(ft_kp, opts_kp, ft_kt, opts_kt, ft_mass, opts_mass);

%%
PropellerFit = pf;
save PropellerFit.mat PropellerFit;