clear all
close all
clc


addpath(genpath('data'))
addpath(genpath('functions'))
addpath(genpath('LPVToolsV1.0')); % Biblioteca LPV


load lpv_model.mat

sys_design = prepare_lpv_model(sys_latdir_trunc,phi_bar,phi_til);
Mach_rate = [-0.05 0.05];
Altitude_rate = [-10 10];
P_lpv = augmented_plant(sys_design,Mach_vec,Altitude_vec,Mach_rate,Altitude_rate);


%% Síntese Hinf para cada ponto de operação
tic
for i=1:length(Mach_vec)
    for j=1:(length(Altitude_vec))
        tic
        [Khinf_vec(:,:,i,j),CLteste,gamma_hinf(i,j)] = hinfsyn(P_lpv.Data(:,:,i,j),5,3);
        t_hinf(i,j) = toc;
    end
end


[Mach_v,Altitude_v] = ndgrid(Mach_vec,Altitude_vec);
Khinf_vec.SamplingGrid = struct('Mach',Mach_v,'Altitude',Altitude_v);

% gamma_LTI máximo
gamma_LTI = max(max(gamma_hinf))
Khinf = pss(Khinf_vec,P_lpv.Domain);

save data/Khinf Khinf t_hinf gamma_LTI Khinf_vec t_hinf gamma_hinf
