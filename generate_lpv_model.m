%% Models load
% Mach - linha, Altitude - coluna
clear all
close all

addpath(genpath('linear_models')); % modelos

%% M = 0.65
load lin_data_M065_h7000m.mat
A_lpv(:,:,1,1) = A;
B_lpv(:,:,1,1) = B;
C_lpv(:,:,1,1) = C;
D_lpv(:,:,1,1) = D;

load lin_data_M065_h8500m.mat
A_lpv(:,:,1,2) = A;
B_lpv(:,:,1,2) = B;
C_lpv(:,:,1,2) = C;
D_lpv(:,:,1,2) = D;

load lin_data_M065_h10000m.mat
A_lpv(:,:,1,3) = A;
B_lpv(:,:,1,3) = B;
C_lpv(:,:,1,3) = C;
D_lpv(:,:,1,3) = D;

%% M = 0.7
load lin_data_M070_h7000m.mat
A_lpv(:,:,2,1) = A;
B_lpv(:,:,2,1) = B;
C_lpv(:,:,2,1) = C;
D_lpv(:,:,2,1) = D;

load lin_data_M070_h8500m.mat
A_lpv(:,:,2,2) = A;
B_lpv(:,:,2,2) = B;
C_lpv(:,:,2,2) = C;
D_lpv(:,:,2,2) = D;

load lin_data_M070_h10000m.mat
A_lpv(:,:,2,3) = A;
B_lpv(:,:,2,3) = B;
C_lpv(:,:,2,3) = C;
D_lpv(:,:,2,3) = D;

%% M = 0.75
load lin_data_M075_h7000m.mat
A_lpv(:,:,3,1) = A;
B_lpv(:,:,3,1) = B;
C_lpv(:,:,3,1) = C;
D_lpv(:,:,3,1) = D;

load lin_data_M075_h8500m.mat
A_lpv(:,:,3,2) = A;
B_lpv(:,:,3,2) = B;
C_lpv(:,:,3,2) = C;
D_lpv(:,:,3,2) = D;

load lin_data_M075_h10000m.mat
A_lpv(:,:,3,3) = A;
B_lpv(:,:,3,3) = B;
C_lpv(:,:,3,3) = C;
D_lpv(:,:,3,3) = D;

Mach_vec = [0.65 0.7 0.75];
Altitude_vec = [7500 8500 9500];


tau_engine = 0.5;
tau_t = 0.1;
tau_e = 0.050;
tau_a = 0.050;
tau_r = 0.050;

A_act = [-1/tau_engine 0 0 0 0 0 0 0;
         0 -1/tau_engine 0 0 0 0 0 0;
         0 0 -1/tau_t 0 0 0 0 0;
         0 0 0 -1/tau_e 0 0 0 0;
         0 0 0 0 -1/tau_e 0 0 0;
         0 0 0 0 0 -1/tau_a 0 0
         0 0 0 0 0 0 -1/tau_a 0;
         0 0 0 0 0 0 0 -1/tau_r];
B_act = -A_act;

elim = ones(1,length(A_lpv(:,1,1,1))+length(A_act(:,1)));
elim(7) = 0;
elim(8) = 0;
elim(9) = 0;
elim(10) = 0;
elim(14) = 0;
elim(17) = 0;
elim(41) = 0;
elim(44) = 0;
elim(70) = 0;
elim(71) = 0;
elim(72) = 0;
elim(73) = 0;
elim(74) = 0;


for i = 1:length(Mach_vec)
    for j = 1:length(Altitude_vec)
        pselect = zeros(1,length(A_lpv(1,:,i,j)));
        pselect(9) = 1;
        rselect = zeros(1,length(A_lpv(1,:,i,j)));
        rselect(10) = 1;
        C = [C_lpv(:,:,i,j);pselect;rselect];
        D = [D_lpv(:,:,i,j);zeros(2,length(B_lpv(1,:,i,j)))];
        
        A_full(:,:,i,j) = [A_lpv(:,:,i,j),B_lpv(:,:,i,j); zeros(length(A_act(:,1)),length(A_lpv(1,:,i,j))),A_act];
        B_full(:,:,i,j) = [zeros(length(A_lpv(1,:,i,j)),length(B_lpv(1,:,i,j)));B_act];
        C_full(:,:,i,j) = [C,D];
        D_full(:,:,i,j) = zeros(length(C(:,1)),length(B_lpv(1,:,i,j)));
        
        % Modelo completo
        sys_full(:,:,i,j) = ss(A_full(:,:,i,j),B_full(:,:,i,j),C_full(:,:,i,j),D_full(:,:,i,j));
        
        % Modelo contendo as variaveis de estado láterodirecionais (inclusive a espiral), a
        % dinamica dos atuadores e os 2 primeiros modos aeroelásticos
        % assimétricos.

        sys_latdir(:,:,i,j) = modred(sys_full(:,:,i,j),logical(elim),'Truncate');
        
        %% Seleção das saídas de interesse para a síntese (p, r, ny_cg, ny_cp)

        A = sys_latdir(:,:,i,j).A;
%         eig(A)
        B = sys_latdir(:,:,i,j).B(:,4:8);
        C = [sys_latdir(:,:,i,j).C(1680,:)
             sys_latdir(:,:,i,j).C(1681,:);
             sys_latdir(:,:,i,j).C(1630,:);
             sys_latdir(:,:,i,j).C(1627,:)];

        D = [sys_latdir(:,:,i,j).D(1680,4:8)
             sys_latdir(:,:,i,j).D(1681,4:8);
             sys_latdir(:,:,i,j).D(1630,4:8);
             sys_latdir(:,:,i,j).D(1627,4:8)];
 
        % Alocação de controle (deflexao assimetrica de profundor e aileron)

        T = [-1 0 0;...
             1 0 0;...
             0 -1 0;...
             0 1 0;...
             0 0 1];
        B = B*T;
        D = D*T;
        sys_latdir_syn(:,:,i,j) = ss(A,B,C,D);
    
        %% Adição dos deslocamentos generalizados na saída do modelo
        [V,D_lambda] = eig(A);
        lambda = diag(D_lambda);
        n = length(A);
        k =1;

        % Definição da transformação de similaridade phi_til
        while k <= n
            phi_til_inv(:,k,i,j) = real(V(:,k));
            if imag(lambda(k)) ~= 0
                k = k+1;
                phi_til_inv(:,k,i,j) = imag(V(:,k));
            end
            k = k+1;
        end
        phi_til(:,:,i,j) = pinv(phi_til_inv(:,:,i,j));

        % Aplicação da transformação
        A_til(:,:,i,j) = phi_til(:,:,i,j)*A*phi_til_inv(:,:,i,j);
        B_til(:,:,i,j) = phi_til(:,:,i,j)*B;
        C_til(:,:,i,j) = C*phi_til_inv(:,:,i,j);
            

        % Obtenção da transformação phi_bar
        k = 1;
        while k <= n
            if imag(lambda(k)) == 0
                Ai_til = A_til(k,k,i,j);
                Bi_til = B_til(k,:,i,j);
                phi_bar(k,k,i,j) = 1;
            else
                Ai_til = A_til(k:k+1,k:k+1,i,j);
                Bi_til = B_til(k:k+1,:,i,j);
                bi = Bi_til(:,1);
                aux = inv([bi Ai_til*bi]);
                phi_i = aux(end,:);
                phi_i = phi_i';
                phi_bar(k:k+1,k:k+1,i,j) = [phi_i Ai_til'*phi_i]';
                k = k+1;
            end
            k = k+1;
        end

        % Aplicação da transformação
        A_bar(:,:,i,j) = phi_bar(:,:,i,j)*A_til(:,:,i,j)*inv(phi_bar(:,:,i,j));
        B_bar(:,:,i,j) = phi_bar(:,:,i,j)*B_til(:,:,i,j);
        C_bar(:,:,i,j) = C_til(:,:,i,j)*inv(phi_bar(:,:,i,j));
        D_bar(:,:,i,j) = D;

        sys_bar = ss(A_bar(:,:,i,j),B_bar(:,:,i,j),C_bar(:,:,i,j),D_bar(:,:,i,j));

        %%

        % Construção do novo espaço de estados com os deslocamentos
        % generalizados como saída
        C1 = eye(size(A));
        C = [C1 ;  C_bar(:,:,i,j)];
        D1 = zeros(length(C1(:,1)),length(B_bar(1,:,i,j)));
        D =  [D1 ;  D];
        sys_bar2 = ss(A_bar(:,:,i,j), B_bar(:,:,i,j),C,D);

         %% Truncamento da espiral - novo método
        
        sys_red = modred(sys_bar2,8,'truncate');
        A_trunc = sys_red.A;
        B_trunc = sys_red.B;
        C_trunc =sys_red.C;
        C_trunc(8,:) = [];
        D_trunc = sys_red.D;
        D_trunc(8,:) = [];
        
        sys_latdir_trunc(:,:,i,j) = ss(A_trunc,B_trunc,C_trunc,D_trunc);
    end
end

save data/lpv_model.mat phi_bar phi_til sys_full sys_latdir sys_latdir_syn sys_latdir_trunc Mach_vec Altitude_vec