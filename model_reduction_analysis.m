clear all
close all
clc

generate_lpv_model

A_bar = sys_latdir_trunc(:,:,1,1).A;
B_bar = sys_latdir_trunc(:,:,1,1).B;
C_bar = sys_latdir_trunc(:,:,1,1).C;
D_bar = sys_latdir_trunc(:,:,1,1).D;

sys_1 = ss(A_bar,B_bar,C_bar,D_bar);

T = phi_bar(:,:,1,1)*phi_til(:,:,1,1);
T_inv = inv(T);
T_inv_others = T_inv(:,[1:7 9:13]);

n = 12;

for i=1:n
    T_inv_others_r = T_inv_others;
    T_inv_others_r(i,:) = [];
    T_others_r = inv(T_inv_others_r);
    A_r = T_inv_others_r*A_bar*T_others_r;
    B_r = T_inv_others_r*B_bar;
    C_r = C_bar*T_others_r;
    D_r = D_bar;
    sys_2(:,:,i) = ss(A_r,B_r,C_r,D_r);
end



% doublet de aileron
t = linspace(0,5,20000);

u = zeros(length(t), 3);
u(5000:10000,2) = 1;
% u(500:1000,7) = -1;

u(10001:15000,2) = -1;
% u(1001:1500,7) = 1;

[y_0, tout_0,x_0] = lsim(sys_latdir_syn(:,:,1,1), u, t);   % y is length(t) x n
figure(1)
plot(tout_0, x_0(:,3));
hold on
[y_1, tout_1,eta_1] = lsim(sys_1, u, t);   % y is length(t) x n
x_1 = T_inv_others*eta_1';
plot(tout_1, x_1(3,:));
legend('0','1')

figure(2)
plot(tout_0, x_0(:,4));
hold on
[y_1, tout_1,eta_1] = lsim(sys_1, u, t);   % y is length(t) x n
x_1 = T_inv_others*eta_1';
plot(tout_1, x_1(4,:));
legend('0','1')


for i=5:12
    [y_2(:,:,i), tout_2(:,i),x_2(:,:,i)] = lsim(sys_2(:,:,i), u, t);
    figure(3)
    subplot(2,4,i-4)
    plot(tout_0, x_0(:,3));
    hold on
    plot(tout_2(:,i), x_2(:,3,i));

    figure(4)
    subplot(2,4,i-4)
    plot(tout_0, x_0(:,4));
    hold on
    plot(tout_2(:,i), x_2(:,4,i));
end
figure(3)
legend('0','2')

figure(4)
legend('0','2')