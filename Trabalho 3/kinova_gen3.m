% Definindo a cinemática direta do Kinova Gen3 usando o padrão DH (Denavit-Hartenberg)
clear L
% Definição dos parâmetros DH para cada elo do robô
%            theta    d(m)     a    alpha type offset
L(1) = Link([  0      -0.2848  0      pi/2  0   0], 'standard');
L(2) = Link([  0      -0.0118  0      pi/2  0   pi], 'standard');
L(3) = Link([  0      -0.4208  0      pi/2  0   pi], 'standard');
L(4) = Link([  0      -0.0128  0      pi/2  0   pi], 'standard');
L(5) = Link([  0      -0.3143  0      pi/2  0   pi], 'standard');
L(6) = Link([  0      0        0      pi/2  0   pi], 'standard');
L(7) = Link([  0      0        0      pi    0   pi], 'standard');

% Convertendo os frames do URDF para os frames do padrão DH
% Definição das transformações homogêneas para alinhar os frames
% Frame 1
R1 = rotx(-90);
P1 = [0;0.12837;0];
T1 = SE3(R1, P1);

% Frame 2
R2 = rotx(-90)*rotz(180);
P2 = [0;0.006425; -0.00001];
T2 = SE3(R2, P2);

% Frame 3
R3 = rotx(-90);
P3 = [0;0.21041;0.00005];
T3 = SE3(R3, P3);

% Frame 4
R4 = roty(180)*rotx(-90);
P4 = [0;0.006475;0.00003];
T4 = SE3(R4, P4);

% Frame 5
R5 = rotx(-90);
P5 = [0;0.1059;0.0001];
T5 = SE3(R5, P5);

% Frame 6
R6 = roty(180)*rotx(-90);
P6 = [0;-0.00007505;-0.00003];
T6 = SE3(R6, P6);

% Frame 7
R7 = rotx(180);
P7 = [0;-0.0002501;0.10596];
T7 = SE3(R7, P7);

% Definição dos parâmetros dinâmicos do robô
% Massa, centro de massa e inércia para cada elo
L(1).m = 1.377; % Massa do shoulder_link
cmass1 = T1.T*[-2.3E-05; -0.010364; -0.07336; 1];
L(1).r = cmass1(1:3); % Centro de massa
L(1).I = [0.00457, 0.004831, 0.001409, 1E-06, 0.000448, 2E-06]; % Inércia

L(2).m = 1.163; % Massa do half_arm_1_link
cmass2 = T2.T*[-4.4E-05; -0.09958; -0.013278; 1];
L(2).r = cmass2(1:3);
L(2).I = [0.011088, 0.001072, 0.011255, 5E-06, -0.000691, 0];

L(3).m = 1.163; % Massa do half_arm_2_link
cmass3 = T3.T*[-4.4E-05; -0.006641; -0.117892; 1];
L(3).r = cmass3(1:3);
L(3).I = [0.010932, 0.011127, 0.001043, 0, 0.000606, -7E-06];

L(4).m = 0.93; % Massa do forearm_link
cmass4 = T4.T*[-1.8E-05; -0.075478; -0.015006; 1];
L(4).r = cmass4(1:3);
L(4).I = [0.008147, 0.000631, 0.008316, -1E-06, -0.0005, 0];

L(5).m = 0.678; % Massa do spherical_wrist_1_link
cmass5 = T5.T*[1E-06; -0.009432; -0.063883; 1];
L(5).r = cmass5(1:3);
L(5).I = [0.001596, 0.001607, 0.000399, 0, 0.000256, 0];

L(6).m = 0.678; % Massa do spherical_wrist_2_link
cmass6 = T6.T*[1E-06; -0.045483; -0.00965; 1];
L(6).r = cmass6(1:3);
L(6).I = [0.001641, 0.00041, 0.001641, 0, -2E-06, 0];

L(7).m = 0.364; % Massa do bracelet_link + end-effector
cmass7 = T7.T*[-9.3E-05; 0.000132; -0.022905; 1];
L(7).r = cmass7(1:3);
L(7).I = [0.000214, 0.000223, 0.00024, 0, -2E-06, 1E-06];

% Inicializar a inércia do motor para todas as juntas como zero
for i = 1:7
    L(i).Jm = 0;
end

% Criando o modelo dinâmico do robô Kinova Gen3
kinova = SerialLink(L, 'name', 'Kinova Gen3', 'comment', 'AK&B');

% Definir a transformação base do robô
Tb0 = trotx(180);
kinova.base = Tb0;

% Inicializar a matriz M_bar como uma matriz diagonal
M_bar = zeros(7, 7);

% Calcular a matriz de inércia diagonal M_bar
for i = 1:7
    % Inicializar o termo diagonal da junta i
    M_bar_i = 0;
    % Acumulador para a distância ao longo das juntas
    distancia_acumulada = 0;
    for j = i:7
        % Adicionar o termo de inércia e centro de massa
        M_bar_i = M_bar_i + L(j).I(3,3) + L(j).m * (norm(L(j).r)^2 + (distancia_acumulada));

        % Distância acumulada até a junta atual
        distancia_acumulada = distancia_acumulada + abs(L(j).d)^2; % Somar comprimento da junta
    end
    % Preencher o elemento diagonal correspondente
    M_bar(i, i) = M_bar_i;
end