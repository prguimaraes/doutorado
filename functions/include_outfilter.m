function [A_new,B_new,C_new, D_new] = include_outfilter(A,B,C,D)
    
    % Including actuators
    tau_p = 1/50;
    tau_r = 1/50;
    tau_aycg = 1/50;
    tau_aycp = 1/50;
    
    tau_inv = [1/tau_p 0 0 0 
             0 1/tau_r 0 0;
             0 0 1/tau_aycg 0;
             0 0 0 1/tau_aycp];

    n_y_y = length(tau_inv(:,1));

    n_x = length(A(1,:));
    n_u = length(B(1,:));

    C_eta = C(1:end-4,:);
    C_y = C(end-3:end,:);

    n_y_eta = length(C_eta(:,1));

    D_eta =D(1:end-4,:);
    D_y = D(end-3:end,:);


    A_new = [A,zeros(n_x,n_y_y); tau_inv*C_y, -tau_inv];
    B_new = [B; tau_inv*D_y];
    C_new = [C_eta, zeros(n_y_eta,n_y_y); zeros(n_y_y,n_x), eye(n_y_y)];
    D_new = [D_eta; zeros(n_y_y,n_u)];