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

% Parâmetros da orientação (roll, pitch, yaw) em radianos
roll = pi;      % Roll
pitch = 0;      % Pitch
yaw = pi/2;     % Yaw

% Matriz de orientação (Rpy)
Rpy = trotx(roll) * troty(pitch) * trotz(yaw);

% Lista de waypoints (zigzag)
waypoints = [
    0.04,  0,    0;
   -0.04,  0,    0;
   -0.04,  0.01, 0;
    0.04,  0.01, 0;
    0.04,  0.02, 0;
   -0.04,  0.02, 0;
   -0.04,  0.03, 0;
    0.04,  0.03, 0;
    0.04,  0.04, 0;
   -0.04,  0.04, 0
];

% Ângulos das juntas iniciais
theta_t = [pi/4, 0];  % Ângulos da mesa posicionadora

% Parâmetros da cinemática inversa
tol = 1e-6;            % Tolerância
lambda = 0.1;          % Parâmetro de regularização
max_iters = 1000;       % Número máximo de iterações

% Inicializar os resultados
joint_angles = zeros(size(waypoints, 1), robot.n); % Ângulos das juntas
T_results = cell(size(waypoints, 1), 1);          % Transformações

q0 = [pi/4, 0, 0, -pi/2, pi/2, 0, 0, 0]; % Configuração inicial
for i = 1:size(waypoints, 1)
    % Criar a matriz de transformação homogênea do waypoint
    T_target = transl(waypoints(i, :)) * Rpy;

    % Resolver cinemática inversa
    q_sol = robot.ikine(T_target, 'tol', tol, 'q0', q0, 'lambda', lambda, 'maxiters', max_iters);

    if isempty(q_sol)
        fprintf('Waypoint %d: solução não encontrada!\n', i);
        joint_angles(i, :) = NaN; % Indicar solução inválida
        T_results{i} = NaN;      % Indicar solução inválida
    else
        joint_angles(i, :) = q_sol;
        T_results{i} = robot.fkine(q_sol);
        q0 = q_sol; % Atualizar q0 para o próximo waypoint
    end
end

% Validar as soluções
for i = 1:size(waypoints, 1)
    % Transformação esperada
    T_expected = transl(waypoints(i, :)) * Rpy;

    % Transformação obtida
    T_obtained = T_results{i};

    % Diferença entre as transformações
    error = norm(T_expected.t - T_obtained.t);

    % Exibir validação
    fprintf('Waypoint %d:\n', i);
    fprintf('Erro de posição: %.6f m\n', error);
    disp('Transformação obtida:');
    disp(T_obtained.T);
end

% Visualizar a trajetória
figure;
robot.plot(joint_angles, 'workspace', [-0.5 0.5 -0.5 0.5 -0.5 0.5]);
