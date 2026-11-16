function [simout] = run_pdoublet_lin(K,Mach,Altitude,interp_method)
 %%
    % Loading and preparating linear model
    Mach_string = string(Mach*100);
    Mach_string = erase(Mach_string,'.');
    Altitude_string = string(Altitude);
    file_string = 'linear_models/lin_data_M0' + Mach_string + '_h' + Altitude_string +'m.mat';
    load(file_string);
    [A,B,C,D] = include_outputs(A,B,C,D);
    [A,B,C,D] = include_actuators(A,B,C,D);

    % Selecting controllers
    K = lpvinterp(K,'Altitude',Altitude,'Mach',Mach,'AltitudeDot',0,'MachDot',0,interp_method);
    K = K.Data;
    
    T = [-1 0 0;...
         1 0 0;...
         0 -1 0;...
         0 1 0;...
         0 0 1];
    
    load_system('pdoublet_lin');
    simout = sim('pdoublet_lin', 'SrcWorkspace', 'current');
    close_system('pdoublet_lin')
    
   