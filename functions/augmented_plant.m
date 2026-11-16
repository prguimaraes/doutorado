function P_lpv = augmented_plant(G_vec,Mach_vec,Altitude_vec,Mach_rate,Altitude_rate)


%% Ganhos
We_1_k = 1; % Tracking, disturbance rejection
We_2_k = 1; % Robustness, control effort
We_3_k = 1; % Damping
Wd_1_k = 1; % Disturbance rejection, damping
Wd_2_k = 1; % Robustness, control effort
Wr_k = 1; % Tracking, control effort

%% Definição dos pesos
% Segundo o artigo

%We_1
% We_1_p = W_s(1,7.5,15,0.1);
We_1_p = W_s(1,100,4,0.9);
We_1_others_k = 0.9;

We_1_p = We_1_k*We_1_p;
We_1_p.InputName = 'We_1_p_u';
We_1_p.OutputName = 'e1_p';

We_1_others = We_1_k*tf(1,1)*[We_1_others_k, 0, 0; ...
               0, We_1_others_k, 0; ...
               0, 0, We_1_others_k];
    
We_1_others.InputName = 'z_rest';
We_1_others.OutputName = 'e1_others';

%We_2
We_2_e = W_s(2.5,0.5,10,100);
We_2_a = W_s(2.5,0.5,10,100);
We_2_r = W_s(2.5,0.5,10,100);



We_2 = We_2_k*[We_2_e, 0, 0; ...
        0, We_2_a, 0; ...
        0, 0, We_2_r];
    
We_2.InputName = 'u';
We_2.OutputName = 'e2';

% We_3
We_3_DR = tf(1,1);
We_3_E2 = tf(0.5,1);
We_3_E5 = tf(0.5,1);

We_3 = We_3_k*[0, 0, 0, 0, 0, 0, We_3_DR; ...
        0, 0, 0, We_3_E2, 0, 0, 0; ...
        0, We_3_E5, 0, 0, 0, 0, 0];


We_3.InputName = 'eta';
We_3.OutputName = 'e3';


% Wd_1
% Wd_1 = Wd_1_k*tf(0.2,1)*eye(3);
Wd_1 = Wd_1_k*[tf(0.05,1) 0 0;
               0 tf(0.2,1) 0;
               0 0 tf(0.1,1)];

Wd_1.InputName = 'd1';
Wd_1.OutputName = 'Wd_1_y';

% Wd_2

% Wd_2 = Wd_2_k*tf(0.2,1)*eye(4);
Wd_2 = Wd_2_k*[tf(0.2,1) 0 0 0;
               0 tf(0.1,1) 0 0;
               0 0 tf(0.1,1) 0;
               0 0 0 tf(0.1,1)];

    
Wd_2.InputName = 'd2';
Wd_2.OutputName = 'Wd_2_y';


% Wr = We_1_p*Wr_k/We_1_k;
Wr = Wr_k*W_s(1,7.5,15,0.1);


Wr.InputName = 'r';
Wr.OutputName = 'Wr_y';

%% Construção da planta aumentada

G_vec.InputName = 'v';
G_vec.OutputName = 'y_total';
% 
% G_vec_denser.InputName = 'v';
% G_vec_denser.OutputName = 'y_total';
% 
eta_select = tf(1,1)*[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; ...
                      0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; ...
                      0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; ...
                      0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; ...
                      0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; ...
                      0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; ...
                      0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0];
eta_select.InputName = 'y_total';
eta_select.OutputName = 'eta';
          
y_select = tf(1,1)*[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0; ...
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0; ...
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0; ...
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1];
y_select.InputName = 'y_total';
y_select.OutputName = 'y';

% Z é Y depois da perturbação
Sum1 = sumblk('z = Wd_2_y - y',4);
p_select = tf(1,1)*[1 0 0 0];
p_select.InputName = 'z';
p_select.OutputName = 'z_p';

rest_select = tf(1,1)*[0 1 0 0;
                       0 0 1 0;
                       0 0 0 1];
rest_select.InputName = 'z';
rest_select.OutputName = 'z_rest';
Sum2 = sumblk('We_1_p_u = Wr_y + z_p',1);
Sum3 = sumblk('v = u + Wd_1_y',3);


% Planta aumentada
P = connect(G_vec,Wr,We_1_p,We_1_others,p_select,rest_select,We_2,We_3,Wd_1,Wd_2,eta_select,y_select,Sum1,Sum2,Sum3,{'r','d1','d2','u'},{'e1_p','e1_others','e2','e3','Wr_y','z'});


%% Definição do domínio e das variações

Mach = pgrid('Mach',Mach_vec,Mach_rate);
Altitude = pgrid('Altitude',Altitude_vec,Altitude_rate);
Domain = rgrid(Mach,Altitude);

% Criação do modelo inicial LPV
% obs: pss - parameter varying state space
% G_lpv = pss(G_vec(:,:,Mach_select,Altitude_select),Domain);
% 
% sys_full_lpv = pss(sys_full(:,:,Mach_select,Altitude_select),Domain);
% 
% rsys_spiral_lpv = pss(rsys_spiral(:,:,Mach_select,Altitude_select), Domain);

P_lpv = pss(P,Domain);