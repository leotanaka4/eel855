% Parâmetros DH do robô
L1 = Revolute('d', 0, 'a', 0, 'alpha', -pi/2, 'offset', -pi/2);   % Elo 1
L2 = Revolute('d', 0.22, 'a', 0, 'alpha', 0, 'offset', pi/2);     % Elo 2

% Definir o robô
kukakp2 = SerialLink([L1 L2], 'name', 'Mesa Posicionadora');

% Transformação homogênea da base para o eixo 0 (T_b0)
T_b0 = [0,  0, -1,  0;
        0,  1,  0,  0;
        1,  0,  0,  0.865;
        0,  0,  0,  1];

% Transformação homogênea do eixo 2 para o efetuador (T_2e)
T_2e = eye(4); % Matriz identidade, conforme especificado

% Incorporar as transformações no robô
kukakp2.base = SE3(T_b0);  % Define a base do robô
robot_kp2.tool = SE3(T_2e);  % Define a ferramenta (tool) do robô

% Configuração inicial das juntas (todas em zero)
q0 = zeros(1, 2);

% Calcular a transformação homogênea do efetuador em relação à base (T_be)
T_be = kukakp2.fkine(q0);

% Exibir as transformações
disp('Transformação da base para o efetuador (T_be):');
disp(T_be.T);

% Plotar o robô na configuração inicial com controles interativos
figure;
kukakp2.teach(q0, 'workspace', [-0.5 0.5 -0.5 0.5 -0.5 1.5]);
