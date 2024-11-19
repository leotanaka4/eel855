% Limpar o ambiente
clear; clc;

% Parâmetros DH do sistema KP90-KP2
L1 = Revolute('d', 0.22, 'a', 0, 'alpha', pi/2, 'offset', pi/2);          % Elo 1
L2 = Revolute('d', 1.73923, 'a', -0.39198, 'alpha', -pi/2, 'offset', 0);  % Elo 2
L3 = Revolute('d', -0.27360, 'a', 0.35, 'alpha', pi/2, 'offset', pi/2);   % Elo 3
L4 = Revolute('d', 0, 'a', 1.35, 'alpha', 0, 'offset', 0);                % Elo 4
L5 = Revolute('d', 0, 'a', 0.041, 'alpha', -pi/2, 'offset', pi/2);        % Elo 5
L6 = Revolute('d', -1.2, 'a', 0, 'alpha', pi/2, 'offset', 0);             % Elo 6
L7 = Revolute('d', 0, 'a', 0, 'alpha', -pi/2, 'offset', 0);               % Elo 5
L8 = Revolute('d', -0.215, 'a', 0, 'alpha', -pi, 'offset', 0);            % Elo 6

% Definir o robô
robot = SerialLink([L1 L2 L3 L4 L5 L6 L7 L8], 'name', 'KP90-KP2');

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
T_8e = T_translation * R_yaw * R_pitch * R_roll;

% Incorporar as transformações no robô
robot.base = trotx(180);  % Define a base do robô
robot.tool = T_8e;  % Define a ferramenta (tool) do robô

% Configuração inicial das juntas (todas em zero)
q0 = zeros(1, 8);

% Calcular a transformação homogênea do efetuador em relação à base (T_be)
T_be = robot.fkine(q0);

% Exibir as transformações
disp('Transformação da base para o efetuador (T_be):');
disp(T_be.T);

% Plotar o robô na configuração inicial com controles interativos
figure;
robot.teach(q0, 'workspace', [-4 4 -4 4 -4 4]);