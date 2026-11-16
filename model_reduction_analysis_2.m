clear all
close all
clc

run("LPVTools\addlpv.m")
load data/lpv_model.mat


% for i=1:3
%     for j=1:3
%         T(:,:,i,j) = phi_bar(:,:,i,j)*phi_til(:,:,i,j);
%         S(:,:,i,j) = inv(T(:,:,i,j));
%         T_r(:,:,i,j) = S([1:7 9:13],:,i,j);
%         S_r(:,:,i,j) = S(:,[1:7 9:13],i,j);
%     end
% end
% 
% 
% S_r_ref = S_r(:,:,2,2);
% 
% for i=1:3
%     for j=1:3
%         T_i_ref(:,:,i,j) = T_r(:,:,i,j)*S_r_ref;
%         T_ref_i(:,:,i,j) = inv(T_i_ref(:,:,i,j));
%         A_bar_r(:,:,i,j) = sys_latdir_trunc(:,:,i,j).A;
%         B_bar_r(:,:,i,j) = sys_latdir_trunc(:,:,i,j).B;
%         C_bar_r(:,:,i,j) = sys_latdir_trunc(:,:,i,j).C;
%         D_bar(:,:,i,j) = sys_latdir_trunc(:,:,i,j).D;
% 
%         A_ref(:,:,i,j) = T_ref_i(:,:,i,j)*A_bar_r(:,:,i,j)*T_i_ref(:,:,i,j);
%         B_ref(:,:,i,j) = T_ref_i(:,:,i,j)*B_bar_r(:,:,i,j);
%         C_ref(:,:,i,j) = C_bar_r(:,:,i,j)*T_i_ref(:,:,i,j);
% 
%         sys_ref(:,:,i,j) = ss(A_ref(:,:,i,j),B_ref(:,:,i,j),C_ref(:,:,i,j),D_bar(:,:,i,j));
%     end
% end



sys_ref = convert_to_ref(sys_latdir_trunc,phi_bar,phi_til);

% % doublet de aileron
t = linspace(0,5,20000);

u = zeros(length(t), 3);
u(5000:10000,2) = 1;

u(10001:15000,2) = -1;


for i=1:3
    for j=1:3
        [y_0, tout_0,x] = lsim(sys_latdir_syn(:,:,i,j), u, t);
        [y_1, tout_1,eta] = lsim(sys_latdir_trunc(:,:,i,j), u, t);   % y is length(t) x n
        [y_2, tout_2,eta_ref] = lsim(sys_ref(:,:,i,j), u, t);   % y is length(t) x n
        
        figure(1)
        subplot(3,3,(i-1)*3+j)
        hold on
        plot(tout_0,y_0(:,1));
        plot(tout_1,y_1(:,1+12));
        plot(tout_2,y_2(:,1+12));
        
        for k=1:12                
            % figure((k*2))
            figure(k+1)
            subplot(3,3,(i-1)*3+j)
            hold on
            plot(tout_1,eta(:,k));

            % figure((k*2+1))
            % subplot(3,3,(i-1)*3+j)
            plot(tout_2,eta_ref(:,k));
        end


    end
end