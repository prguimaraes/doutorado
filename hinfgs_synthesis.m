clear all
close all
clc

run("LPVTools\addlpv.m")
addpath(genpath('data'))
addpath(genpath('functions'))


load lpv_model.mat

sys_design = prepare_lpv_model(sys_latdir_trunc,phi_bar,phi_til);
Mach_rate = [-0.05 0.05];
Altitude_rate = [-10 10];
P_lpv = augmented_plant(sys_design,Mach_vec,Altitude_vec,Mach_rate,Altitude_rate);

P_vec = [];
for i_M = 1:length(Mach_vec)
    for i_alt = 1: length(Altitude_vec)
        P = P_lpv.Data(:,:,i_M,i_alt);
        P_vec = [P_vec ltisys(P.A,P.B,P.C,P.D,eye(size(P.A)))];
    end
end
pdP = psys(P_vec);
r = [5,3];
[gopt,pdK,R,S] = hinfgs(pdP,r,0);

for i_M=1:3
    for i_Alt=1:3
        j = (i_M-1)*3 + i_Alt;
        Khinfgs(:,:,i_M,i_Alt) = ss(psinfo(pdK,'sys',j));
    end
end

save data/Khinfgs Khinfgs gopt pdK R S