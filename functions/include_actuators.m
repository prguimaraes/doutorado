function [A_new,B_new,C_new, D_new] = include_actuators(A,B,C,D)
    
    % Including actuators
    tau_engine = 0.5;
    tau_t = 0.1;
    tau_e = 0.050;
    tau_a = 0.050;
    tau_r = 0.050;
    
    A_act = [-1/tau_engine 0 0 0 0 0 0 0;
             0 -1/tau_engine 0 0 0 0 0 0;
             0 0 -1/tau_t 0 0 0 0 0;
             0 0 0 -1/tau_e 0 0 0 0;
             0 0 0 0 -1/tau_e 0 0 0;
             0 0 0 0 0 -1/tau_a 0 0
             0 0 0 0 0 0 -1/tau_a 0;
             0 0 0 0 0 0 0 -1/tau_r];
    B_act = -A_act;

    A_new = [A,B; zeros(length(A_act(:,1)),length(A(1,:))),A_act];
    B_new = [zeros(length(A(1,:)),length(B(1,:)));B_act];
    C_new = [C,D];
    D_new = zeros(length(C(:,1)),length(B(1,:)));