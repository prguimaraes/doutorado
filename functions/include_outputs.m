function [A_new,B_new,C_new, D_new] = include_outputs(A,B,C,D)
    % Including p and r as outputs
    pselect = zeros(1,length(A(1,:)));
    pselect(9) = 1;
    rselect = zeros(1,length(A(1,:)));
    rselect(10) = 1;
    A_new = A;
    B_new = B;
    C_new = [C;pselect;rselect];
    D_new = [D;zeros(2,length(B(1,:)))];