%% Implementation 1: Infinite Time Horizon w/o Known Input, No Integrators
% See WeeklyReport_10052021

sys_name = 'QuadRotor_Simulink_LQT';

load_system(sys_name);
sys_wks = get_param(sys_name, 'ModelWorkspace');
assignin(sys_wks, "QR_Object", qrs);

%% Controller
A = qrs.LinearPlantModel.A;
B = qrs.LinearPlantModel.B;
%C = qrs.LinearPlantModel.C;
%C = [zeros(6,4), eye(6), zeros(6,6)];
C = eye(size(A));

CAll = eye(size(A)); 

% We're going to let C be all the states now because we need to stabilize and track the reference.  
% With only position outputs, the associated Hamiltonian matrix has
% eigenvalues on the imaginary axis.  
D = zeros(size(C,1),size(B,2));
DAll = zeros(size(CAll,1), size(B,2));

%% Weighting Matrices
P = eye(size(C,1)); % For not
Q = eye(size(C,1));
Q(1:4,1:4) = zeros(4); % Speed States
Q(8:10, 8:10) = zeros(3); % Velocity States
Q(11:13, 11:13) = 0.01*eye(3); % RPY
Q(14:16, 14:16) = 0.01*eye(3); % Angular Velocity 

rho = 0.0001;
R = rho*eye(size(B,2));

%% Process Trajectory
Times = qrs.RefTraj.TimeSeries.Time;
Tf = Times(end);
r_data = qrs.RefTraj.TimeSeries.Data;
r_data = permute(r_data,[3 1 2])';
r = @(t_s) [interp1(Times', r_data(1,:), t_s);...
    interp1(Times', r_data(2,:), t_s);...
    interp1(Times', r_data(3,:), t_s)];

%% Offline Calculations

% Get K Gain
[Sstatic,Kstatic] = solveLQTInfinite(A,B,C,Q,R);

[S, K, tk] = solveLQRFinite(A,B,C,Q,R,P,Tf);
% Interpolate K over time
Kcol = reshape(K,size(K,1)*size(K,2),size(K,3), 1);


Kfun = @(t) reshape(interp1(tk',Kcol',t), size(K,1), size(K,2));

%% Calculate nu_0 by integrating backwards

nu_Tf = C'*P*[zeros(4,1); r(Tf); zeros(9,1)];

% Integrate to find nu
[t,nu] = ode45(@(s,nu) nuODE(s,nu, A, B, Kstatic, C, Q, r), [Tf, 0], nu_Tf);

nu_0 = nu(end,:)';


%% Simulate
 simout = sim(sys_name);
 qrso = QRSimOut(simout, qrs.QR.BM);

function [S,K] = solveLQTInfinite(A,B,C,Q,R)
    [S,K] = ssRicatti();

    function [S,K] = ssRicatti()
        %rhs = A'*S + S*A - S*B*inv(R)*B'*S+C'*Q*C;
        Qmod = C'*Q*C;
        E = eye(size(A));
        G = zeros(size(A));
        [S,K,~] = icare(A,B,Qmod,R, zeros(size(B)),E,G);
    end
end

function [S,K,t] = solveLQRFinite(A,B,C,Q,R,P,Tf)
    Sf = C'*P*C;
    [t,Svec] = ode45(@ricattiRHS, [Tf, 0], Sf);

    S = zeros(16,16,numel(t));
    K = zeros(size(B,2), size(A,1), numel(t));
    % Transform S to array of matrices with time in third dimension
    for i = 1:numel(t)
        v = Svec(i,:)'; % Convert to column vector
        S(:,:,i) = reshape(v,size(A));
        K(:,:,i) = inv(R)\(B'*S(:,:,i));
    end
    function sdot = ricattiRHS(s,S)
        S = reshape(S, size(A));
        sdot = -(A'*S + S*A - S*B*inv(R)*B'*S+C'*Q*C);
        sdot = sdot(:);
    end   
end

function nudot = nuODE(s,nu, A, B, K, C, Q, r)
if isa(K, 'function_handle')
    K_ = K(s);
else
    K_ = K;
end
    nudot = -((A - B*K_)'*nu + C'*Q*[zeros(4,1); r(s); zeros(9,1)]);
end



