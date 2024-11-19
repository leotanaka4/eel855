% Limpar o ambiente
clear; clc;

% Parâmetros DH do robô
L1 = Revolute('d', -0.675, 'a', 0.35, 'alpha', pi/2, 'offset', 0);      % Elo 1
L2 = Revolute('d', 0, 'a', 1.35, 'alpha', 0, 'offset', 0);              % Elo 2
L3 = Revolute('d', 0, 'a', 0.041, 'alpha', -pi/2, 'offset', pi/2);      % Elo 3
L4 = Revolute('d', -1.2, 'a', 0, 'alpha', pi/2, 'offset', 0);           % Elo 4
L5 = Revolute('d', 0, 'a', 0, 'alpha', -pi/2, 'offset', 0);             % Elo 5
L6 = Revolute('d', -0.215, 'a', 0, 'alpha', -pi, 'offset', 0);          % Elo 6

% Definir o robô
robot = SerialLink([L1 L2 L3 L4 L5 L6], 'name', 'Braço Robótico');

% Transformação homogênea da base para o eixo 0 (T_b0)
T_b0 = [1,  0,  0,  0;
        0, -1,  0,  0;
        0,  0, -1,  0;
        0,  0,  0,  1];

% Parâmetros de posição (xyz)
x = 0.03046;
y = 0.033;
z = 0.43161;

% Parâmetros de orientação (rpy) em graus (convertendo para radianos)
roll = -13.5;  % Roll
pitch = 51.5;  % Pitch
yaw = -5.4;    % Yaw

% Matriz de translação
T_translation = transl(x, y, z);

% Matrizes de rotação para roll, pitch e yaw
R_roll = trotx(roll);
R_pitch = troty(pitch);
R_yaw = trotz(yaw);

% Transformação homogênea total
T_6e = T_translation * R_yaw * R_pitch * R_roll;

% Incorporar as transformações no robô
robot.base = SE3(T_b0);  % Define a base do robô
robot.tool = T_6e;  % Define a ferramenta (tool) do robô

% Configuração inicial das juntas (todas em zero)
q0 = zeros(1, 6);

% Calcular a transformação homogênea do efetuador em relação à base (T_be)
T_be = robot.fkine(q0);

% Exibir a transformação final do efetuador
disp('Transformação do efetuador em relação à base (T_be):');
disp(T_be.T);

% Plotar o robô na configuração inicial com controles interativos
figure;
robot.teach(q0, 'workspace', [-5 5 -5 5 -5 5]);