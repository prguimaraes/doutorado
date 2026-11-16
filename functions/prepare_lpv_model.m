function [sys_design] = prepare_lpv_model(sys_latdir_trunc,phi_bar,phi_til)
    sys_ref = convert_to_ref(sys_latdir_trunc,phi_bar,phi_til);
    for i = 1:3
        for j=1:3
            
            [A,B,C,D] = include_outfilter(sys_ref(:,:,i,j).A,sys_ref(:,:,i,j).B,sys_ref(:,:,i,j).C,sys_ref(:,:,i,j).D);
            [A,B,C,D] = include_actuators(A,B,C,D);
            B = B(:,4:8);
            D = D(:,4:8);
            T = [-1 0 0;...
             1 0 0;...
             0 -1 0;...
             0 1 0;...
             0 0 1];
            B = B*T;
            D = D*T;
            elim = ones(1,length(A));
            elim(1) = 0;
            elim(2) = 0;
            elim(3) = 0;
            elim(4) = 0;
            elim(5) = 0;
            elim(6) = 0;
            elim(7) = 0;
            elim(8) = 0;
            elim(9) = 0;
            elim(10) = 0;
            elim(11) = 0;
            elim(15) = 0;
            elim(16) = 0;
            elim(17) = 0;
            elim(18) = 0;
            elim(19) = 0;
            
             C = [C(1:7,:);
                 zeros(5,14), eye(5);
                 C(8:11,:)];
    
            D = [D;
                 zeros(5,3)];
    
            sys = ss(A,B,C,D);
            sys_design(:,:,i,j) = modred(sys,logical(elim),'Truncate');
        end
    end