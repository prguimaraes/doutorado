clear all
close all
clc


addpath(genpath('data'))
addpath(genpath('functions'))
addpath(genpath('simulink'))
addpath(genpath('LPVToolsV1.0')); % Biblioteca LPV

load Khinf.mat
load Klpv_20261020_071116.mat

i_lpv = 5;
interp_method = 'linear';

figure

Altitude_list = [7750 9250];
Mach_list = [0.675 0.725];

for i_alt=1:length(Altitude_list)
    for i_Mach = 1:length(Mach_list)

        Altitude = Altitude_list(i_alt);
        Mach = Mach_list(i_Mach);
        simout = run_pdoublet_lin(Khinf,Mach,Altitude,interp_method);
        t_hinf = simout.tout;
        p_hinf = simout.yout{1}.Values.Data(:,1);

        simout = run_pdoublet_lin(Klpv{i_lpv},Mach,Altitude,interp_method);
        t_lpv = simout.tout;
        p_lpv = simout.yout{1}.Values.Data(:,1);

        n_plot = 220 + i_Mach + length(Mach_list)*(i_alt-1);
        subplot(n_plot)
        plot(t_hinf,p_hinf)
        hold on
        plot(t_lpv,p_lpv)
        title('Altitude = '+ string(Altitude) + ', Mach = ' + string(Mach))
        xlim([0 20]);
        ylim([-1.5 1.5]);

    end
end
