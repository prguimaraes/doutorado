clear all
close all

addpath(genpath('data'))

load lpv_model

sys = sys_latdir(:,:,1,1);
C_states = eye(13);
D_states = zeros(13,8);
C_new = [sys.C; C_states];
D_new = [sys.D; D_states];

sys = ss(sys.A,sys.B,C_new,D_new);



A = sys.A;
B = sys.B;
C = sys.C;
D = sys.D;

[V,D_lambda] = eig(sys.A);
lambda = diag(D_lambda);
n = length(sys.A);
k =1;

% Definição da T_ransformação de similaridade phi_til
while k <= n
    T(:,k) = real(V(:,k));
    if imag(lambda(k)) ~= 0
        k = k+1;
        T(:,k) = imag(V(:,k));
    end
    k = k+1;
end

% T_inv = inv(T);

% A_z= T_inv*A*T;
A_z = T\(A*T);
% B_z = T_inv*B;
B_z = T\B;
C_z = C*T;

Lambda_r = A_z([1:7, 9:13],[1:7, 9:13]);
Lambda_t = A_z(8,8);

B_r = B_z([1:7 9:13],:);
B_t = B_z(8,:);

C_r = C_z(:,[1:7, 9:13]);
C_t = C_z(:,8);

T_r = T(:,[1:7, 9:13]);
T_z = T(:,8);

T_r_cross = (T_r' * T_r) \ T_r';
% T_r_cross = pinv(T_r);


% T_r: n x (n-1) retained modal basis
[n, r] = size(T_r);

assert(r == n-1, 'T_r must have n-1 columns.');

% Normalize each modal vector to remove arbiT_rary eigenvector scaling
T_r = T_r ./ vecnorm(T_r, 2, 1);

% Test every possible physical state to omit
cond_num = zeros(n, 1);
sigma_min = zeros(n, 1);

for i = 1:n
    % Retain every row except row i
    rows = setdiff(1:n, i);
    M = T_r(rows, :);

    % Singular values and condition number
    s = svd(M);
    sigma_min(i) = s(end);
    cond_num(i) = s(1) / s(end);
end

% Sort candidates from best-conditioned to worst-conditioned
[cond_sorted, idx] = sort(cond_num, 'ascend');

results = table(idx, cond_sorted, sigma_min(idx), ...
    'VariableNames', ...
    {'OmittedStateIndex', 'ConditionNumber', 'MinSingularValue'});

disp(results);


T_r_s = T_r(1:12,:);
% T_r_s = T_r([1:3, 5:13], :);
T_r_s_inv = inv(T_r_s);

A_trunc = (T_r_s*Lambda_r)*T_r_s_inv;

A_test = (T_r*Lambda_r)*T_r_cross;

B_trunc = T_r_s*B_r;
C_trunc = C_r*T_r_s_inv;
D_trunc = D;

sys_trunc = ss(A_trunc,B_trunc,C_trunc,D_trunc);

sys_z = ss(Lambda_r,B_r,C_r,D);

% Time vector
t = linspace(0,2,2000);

% Input: unit step on first control only
u = zeros(length(t), 8);
u(500:1000,6) = 1;
u(500:1000,7) = -1;

u(1001:1500,6) = -1;
u(1001:1500,7) = 1;

% Simulate
[y_trunc, tout_trunc,x_trunc] = lsim(sys_trunc, u, t);   % y is length(t) x n
[y, tout,x] = lsim(sys, u, t);   % y is length(t) x n
[y_z, tout_z, x_z] = lsim(sys_z, u, t);   % y is length(t) x n
% Plot first four states (or fewer if n<4)
k = 4
figure;
plot(tout_trunc, y_trunc(:,1689));
hold on
plot(tout, y(:,1689));
plot(tout_z, y_z(:,1689));
xlabel('Time (s)');
ylabel('State value');
grid on;

