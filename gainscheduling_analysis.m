clear all
close all
clc

addpath(genpath('data'))
addpath(genpath('functions'));
addpath(genpath('LPVToolsV1.0')); % Biblioteca LPV

load Khinf.mat
load Klpv_20261020_071116.mat

Khinf_ref = Khinf.Data(:,:,2,2);
i_lpv = 5;
Klpv_ref = Klpv{i_lpv}.Data(:,:,2,2);

for i = 1:3
    for j = 1:3
        dA_hinf = Khinf.Data(:,:,i,j).A - Khinf_ref.A;
        dB_hinf = Khinf.Data(:,:,i,j).B - Khinf_ref.B;
        dC_hinf = Khinf.Data(:,:,i,j).C - Khinf_ref.C;
        dD_hinf = Khinf.Data(:,:,i,j).D - Khinf_ref.D;
        dA_hinf_norm(i,j) = norm(dA_hinf,"fro");
        dB_hinf_norm(i,j) = norm(dB_hinf,"fro");
        dC_hinf_norm(i,j) = norm(dC_hinf,"fro");
        dD_hinf_norm(i,j) = norm(dD_hinf,"fro");

        dA_lpv = Klpv{i_lpv}.Data(:,:,i,j).A - Klpv_ref.A;
        dB_lpv = Klpv{i_lpv}.Data(:,:,i,j).B - Klpv_ref.B;
        dC_lpv = Klpv{i_lpv}.Data(:,:,i,j).C - Klpv_ref.C;
        dD_lpv = Klpv{i_lpv}.Data(:,:,i,j).D - Klpv_ref.D;
        dA_lpv_norm(i,j) = norm(dA_lpv,"fro");
        dB_lpv_norm(i,j) = norm(dB_lpv,"fro");
        dC_lpv_norm(i,j) = norm(dC_lpv,"fro");
        dD_lpv_norm(i,j) = norm(dD_lpv,"fro");      
    end
end