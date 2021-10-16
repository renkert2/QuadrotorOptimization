%% Implementation 1: Infinite Time Horizon w/o Known Input, No Integrators
% See WeeklyReport_10052021

sys_name = 'QuadRotor_Simulink_Aksland';

load_system(sys_name);
sys_wks = get_param(sys_name, 'ModelWorkspace');
assignin(sys_wks, "QR_Object", qrs);

%% Controller
A = qrs.LinearPlantModel.A;
B = qrs.LinearPlantModel.B;
C = qrs.LinearPlantModel.C;
%C = eye(size(A)); 
% We're going to let C be all the states now because we need to stabilize and track the reference.  
% With only position outputs, the associated Hamiltonian matrix has
% eigenvalues on the imaginary axis.  
D = zeros(size(C,1),size(B,2));


%% Weighting Matrices
P = zeros(size(C,1)); % For not
Q = eye(size(A));
R = eye(size(B,2));

K = lqr(A,B,Q,R);

K_tilde = -inv(C*inv(A-B*K)*B);

%% Offline Calculations
% Get K Gain
[S,K] = solveLQTInfinite(A,B,C,Q,R);

function [S,K] = solveLQTInfinite(A,B,C,Q,R)
    [S,K] = ricatti();

    function [S,K] = ricatti()
        %rhs = A'*S + S*A - S*B*inv(R)*B'*S+C'*Q*C;
        Qmod = C'*Q*C;
        E = eye(size(A));
        G = zeros(size(A));
        [S,K,~] = icare(A,B,Qmod,R, zeros(size(B)),E,G);
    end
end




