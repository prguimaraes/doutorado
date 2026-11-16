clear all
close all
clc

addpath(genpath('data'))
addpath(genpath('functions'));
run("LPVTools\addlpv.m")

load lpv_model.mat

Mach_rate = [-0.05 0.05];
Altitude_rate = [-10 10];
sys_design = prepare_lpv_model(sys_latdir_trunc,phi_bar,phi_til);
% sys_design=sys_latdir_trunc
P_lpv = augmented_plant(sys_design,Mach_vec,Altitude_vec,Mach_rate,Altitude_rate);

%% Síntese LPV


% % Aumentar numero de iterações e testar o linear
% max_iter = 600;
% feasp = -1;
% trace = 0;
% mincx_options = zeros(5,1);
% mincx_options(2) = max_iter;
% % mincx_options(3) = feasp;
% mincx_options(5) = trace;
% % mincx_options(4) = 1;
% 
% % opt = lpvsynOptions('Method','MaxFeas','SolverOptions',mincx_options,'Gammalb',gamma_LTI,'Gammaub',1.3*gamma_LTI);
% % opt = lpvsynOptions('Method','MinGamma','Solver','sedumi');
opt = lpvsynOptions('BackOffFactor',1.02);
% % tic
% % testar: [gopt,pdK,R,S] = hinfgs(pdP,r,gmin,tol,tolred)
% % testar tambem: hinfstruct
% % toc


Mach = pgrid('Mach',Mach_vec,Mach_rate);
Altitude = pgrid('Altitude',Altitude_vec,Altitude_rate);

b1 = basis(1,'Mach',0,'Altitude',0);
% 
b_M = basis(Mach,'Mach',1,'Altitude',0);
b_h = basis(Altitude,'Mach',0,'Altitude',1);
b_Mach = (1 - b_M^2)^(-1/2);
% 
Xb{1} = b1;
Xb{2} = [b1;b_M,b_h];
Xb{3} = [b1;b_M,b_h,b_M*b_h];
Xb{4} = [b1;b_M,b_h,b_M*b_h,b_M^2,b_h^2];
Xb{5} = [b1;b_M;b_h;b_M*b_h;b_M^2;b_h^2;b_M*b_h^2;b_M^2*b_h;b_M^2*b_h^2];
Xb{6} = [b1;b_M,b_h,b_M*b_h,b_Mach,b_h^2];
Xb{7} = [b1;b_M;b_h;b_M*b_h;b_Mach;b_h^2;b_M*b_h^2;b_Mach*b_h;b_Mach*b_h^2];
Xb{8} = [b1, b_M, b_h,b_Mach];

sel_basis = [1];

for i = 1:length(sel_basis)
    basis = sel_basis(i);
    Xb_i = Xb{basis};
    Yb_i = Xb{basis};
    tic
    [Klpv{basis},gam{basis},info{basis}] = lpvsyn(P_lpv,5,3,Xb_i,Yb_i,opt)
    t{basis} = toc
end


ts = char(datetime('now','Format','yyyyMMdd_HHmmss'));
filename = ['data/Klpv_', ts];
save(filename,'Klpv','gam','info','t','Xb','sel_basis')
