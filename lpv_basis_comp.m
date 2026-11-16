clear all
close all
clc

clear all
close all
clc


addpath(genpath('data'))
addpath(genpath('functions'))
addpath(genpath('simulink'))
addpath(genpath('LPVToolsV1.0')); % Biblioteca LPV

load Klpv_20261020_071116.mat

interp_method = 'nearest';

figure
hold on

Altitude = 8500;
Mach = 0.7;

lpv_list = [1 2 3 4 5];

for i_lpv=1:length(lpv_list)
    

    lpv = lpv_list(i_lpv);
    simout = run_pdoublet_lin(Klpv{lpv},Mach,Altitude,interp_method);
    t_lpv = simout.tout;
    p_lpv = simout.yout{1}.Values.Data(:,1);
    if (i_lpv == 1)
        y_ref = simout.yout{3}.Values.Data;
        plot(t_lpv,y_ref)
    end

    plot(t_lpv,p_lpv)
    xlim([0 20]);
    ylim([-1.5 1.5]);
end

grid on
l = legend('Command','$K_{LPV1}$ response','$K_{LPV2}$ response','$K_{LPV3}$ response','$K_{LPV4}$ response','$K_{LPV5}$ response');
set(l,'interpreter','Latex');
xlabel('Time [s]','interpreter','Latex')
ylabel('p [deg/s]','interpreter','Latex')

 