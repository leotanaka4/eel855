% Parâmetros DH definidos no enunciado
%            theta       d            a       alpha     type offset
L(1) = Link([  0      -0.2848         0        pi/2      0      0], 'standard');
L(2) = Link([  0      -0.0118         0        pi/2      0     pi], 'standard');
L(3) = Link([  0      -0.4208         0        pi/2      0     pi], 'standard');
L(4) = Link([  0      -0.0128         0        pi/2      0     pi], 'standard');
L(5) = Link([  0      -0.3143         0        pi/2      0     pi], 'standard');
L(6) = Link([  0         0            0        pi/2      0     pi], 'standard');
L(7) = Link([  0         0            0        pi        0     pi], 'standard');

%Transformações URDF to DH
%frame 1
R1 = eye(3);
P1 = [0;0;-0.15643];
T1 = SE3(R1, P1);

%frame 1
R2 = eye(3);
P2 = [0;-0.00001;-0.005375];
T2 = SE3(R2, P2);

%frame 3
R3 = rotz(180);
P3 = [0;-0.00005;-0.21039];
T3 = SE3(R3, P3);

%frame 4
R4 = eye(3);
P4 = [0;0.00003;-0.006325];
T4 = SE3(R4, P4);

%frame 5
R5 = rotz(180);
P5 = [0;0.0001;-0.158];
T5 = SE3(R5, P5);

%frame 6
R6 = eye(3);
P6 = [0;0.05037;-0.00007505];
T6 = SE3(R6, P6);

%frame 7
R7 = rotz(180);
P7 = [0;-0.0002501;-0.05556];
T7 = SE3(R7, P7);


%Dinamica
L(1).m = 1.377; % Mass of shoulder_link
cmass = T1.T*[-2.3E-05; -0.010364; -0.07336; 1];
L(1).r = cmass(1:3); % Center of mass
L(1).I = [0.00457, 0.004831, 0.001409, 1E-06, 0.000448, 2E-06]; % Inertia

L(2).m = 1.163; % Mass of half_arm_1_link
cmass = T2.T*[-4.4E-05; -0.09958; -0.013278; 1];
L(2).r = cmass(1:3); % Center of mass
L(2).I = [0.011088, 0.001072, 0.011255, 5E-06, -0.000691, 0]; % Inertia

L(3).m = 1.163; % Mass of half_arm_2_link
cmass = T3.T*[-4.4E-05; -0.006641; -0.117892; 1];
L(3).r = cmass(1:3); % Center of mass
L(3).I = [0.010932, 0.011127, 0.001043, 0, 0.000606, -7E-06]; % Inertia

L(4).m = 0.93; % Mass of forearm_link
cmass = T4.T*[-1.8E-05; -0.075478; -0.015006; 1];
L(4).r = cmass(1:3); % Center of mass
L(4).I = [0.008147, 0.000631, 0.008316, -1E-06, -0.0005, 0]; % Inertia

L(5).m = 0.678; % Mass of spherical_wrist_1_link
cmass = T5.T*[1E-06; -0.009432; -0.063883; 1];
L(5).r = cmass(1:3); % Center of mass
L(5).I = [0.001596, 0.001607, 0.000399, 0, 0.000256, 0]; % Inertia

L(6).m = 0.678; % Mass of spherical_wrist_2_link
cmass = T6.T*[1E-06; -0.045483; -0.00965; 1];
L(6).r = cmass(1:3); % Center of mass
L(6).I = [0.001641, 0.00041, 0.001641, 0, -2E-06, 0]; % Inertia

L(7).m = 0.364; % Mass of bracelet_link + end-effector
cmass = T7.T*[-9.3E-05; 0.000132; -0.022905; 1];
L(7).r = cmass(1:3); % Center of mass
L(7).I = [0.000214, 0.000223, 0.00024, 0, -2E-06, 1E-06]; % Inertia

for i = 1:7
    L(i).Jm = 0;
end

% Definir a transformação homogênea da base
T_base = trotx(180); % Conversão para radianos

% Criar o robô serial
Kinova = SerialLink(L, 'name', 'KinovaGen3', 'base', T_base);