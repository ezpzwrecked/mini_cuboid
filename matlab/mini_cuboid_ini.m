clc, clear all
addpath app/
%% Open Gui

% run GPA_nucleo_UART_exported.m

% You might want to use the following code later.

% fprintf('Matrix<float, 1, 2> K(%1.4ff, %1.4ff);\n', K2(1), K2(2));
% fprintf('Matrix<float, 1, 2> K4(%1.4ff, %1.4ff, %1.4ff, %1.4ff);\n', K4(1), K4(2), K4(3), K4(4));

% % Parameters for Simulation
% phi1_0 = 0;
% param = get_parameter();
% [A_sim, B_sim, C_sim, D_sim] = linmod('mini_cuboid_simscape_sim');
% sys_sim = minreal( ss(A_sim, B_sim, C_sim, D_sim) );
% 
% phi1_0 = 30 * pi/180;
% K4 = zeros(1,4);


%%

% % --- AUFGABE 2.4 ---
% Ts = 1/1e3;
% tau = 0.1;
% 
% b1 = ;
% b0 = ;
% a0 = ;
% u_kmin1 = 0;
% y_kmin1 = 0;
% 
% t = (0:Ts:1).';
% u = zeros(size(t));
% y = zeros(size(t));
% u(t > 0.1) = 1;
% for i = 1:length(t)
%     % y(k) = b1*u(k) + b0*u(k-1) - a0*y(k-1)
%     y(i) = b1 * u(i) + b0 * u_kmin1 - a0 * y_kmin1;
%     u_kmin1 = u(i);
%     y_kmin1 = y(i);
% end
% 
% figure(1)
% plot(t, [u, y]), grid on
% xlabel('Time (sec)')

%% Zustandsregler ohne Scheibe
p = get_parameter;
p.J1 = 3e-3;

J = p.J1 + p.m*p.R^2;
A = [0, 1; p.m*p.g*p.R/J, 0];
B = [0; -1/J];

P_des2 = [-11+11j, -11-11j];
K2 = place(A,B,P_des2);
fprintf('Matrix<float, 1, 2> K(%1.4ff, %1.4ff);\n', K2(1), K2(2));


%% System 3. Ordnung x3 = Drehzahl Scheibe im globalen System
A3 = [A, zeros(2,1); 0, 0, 0];
B3 = [B; 1/p.J2];
C3 = [1, 0, 0; 0, 1, 0; 0, -1, 1];

s1 = ss(A3,B3,C3,0);
s2 = ss2ss(s1,C3);

%% Erweitere System um Integratorzustand
A4 = [s2.A, zeros(3,1); 0, 0, -1, 0];
B4 = [s2.B; 0];
P_des4 = [-11+5j, -11-5j, -20, -0.2];
K4 = place(A4,B4,P_des4);
fprintf('Matrix<float, 1, 4> K4(%1.4ff, %1.4ff, %1.4ff, %1.4ff);\n', K4(1), K4(2), K4(3), K4(4));
