% ------------------ Configuração do Robô Kuka KR90 ------------------------

clear L

% Parâmetros DH do robô
L(1) = Revolute('d', -0.675, 'a', 0.35, 'alpha', pi/2, 'offset', 0);      % Elo 1
L(2) = Revolute('d', 0, 'a', 1.35, 'alpha', 0, 'offset', 0);              % Elo 2
L(3) = Revolute('d', 0, 'a', 0.041, 'alpha', -pi/2, 'offset', pi/2);      % Elo 3
L(4) = Revolute('d', -1.2, 'a', 0, 'alpha', pi/2, 'offset', 0);           % Elo 4
L(5) = Revolute('d', 0, 'a', 0, 'alpha', -pi/2, 'offset', 0);             % Elo 5
L(6) = Revolute('d', -0.215, 'a', 0, 'alpha', -pi, 'offset', 0);          % Elo 6

% Definir o robô
kukakr90 = SerialLink(L, 'name', 'Braço Robótico');

% Transformação homogênea da base para o eixo 0 (T_b0)
T_b0 = [1,  0,  0,  0;
        0, -1,  0,  0;
        0,  0, -1,  0;
        0,  0,  0,  1];

T_6e = transl(0.03046, 0.033, 0.43161) * trotz(-5.4) * troty(51.5) * trotx(-13.5);

% Incorporar as transformações no robô
kukakr90.base = T_b0;  % Define a base do robô
kukakr90.tool = T_6e;       % Define a ferramenta (tool) do robô

% ---------------- Configuração do Robô Kuka KP2 (Mesa Posicionadora) -----------------

clear L; % Limpar variáveis anteriores de elos

% Definição dos elos
L(1) = Revolute('d', 0, 'a', 0, 'alpha', -pi/2, 'offset', -pi/2);   
L(2) = Revolute('d', 0.22, 'a', 0, 'alpha', 0, 'offset', pi/2);     

% Transformações da base e ferramenta
T_base_kp2 = [0,  0, -1,  0;
              0,  1,  0,  0;
              1,  0,  0,  0.865;
              0,  0,  0,  1];
T_tool_kp2 = eye(4); % Matriz identidade para a ferramenta

% Criação do robô Kuka KP2
kukaKp2 = SerialLink(L, 'name', 'Kp2', 'base', T_base_kp2, 'tool', T_tool_kp2);

% ---------------- Configuração do Sistema KR90-KP2 -----------------------

clear L; % Limpar variáveis anteriores de elos

% Parâmetros DH do sistema KP90-KP2
L(1) = Revolute('d', 0.22, 'a', 0, 'alpha', pi/2, 'offset', pi/2);          % Elo 1
L(2) = Revolute('d', 1.73923, 'a', -0.39198, 'alpha', -pi/2, 'offset', 0);  % Elo 2
L(3) = Revolute('d', -0.27360, 'a', 0.35, 'alpha', pi/2, 'offset', pi/2);   % Elo 3
L(4) = Revolute('d', 0, 'a', 1.35, 'alpha', 0, 'offset', 0);                % Elo 4
L(5) = Revolute('d', 0, 'a', 0.041, 'alpha', -pi/2, 'offset', pi/2);        % Elo 5
L(6) = Revolute('d', -1.2, 'a', 0, 'alpha', pi/2, 'offset', 0);             % Elo 6
L(7) = Revolute('d', 0, 'a', 0, 'alpha', -pi/2, 'offset', 0);               % Elo 5
L(8) = Revolute('d', -0.215, 'a', 0, 'alpha', -pi, 'offset', 0);            % Elo 6

% Definir o robô
kukakr90kp2 = SerialLink(L, 'name', 'KP90-KP2');

T_8e = transl(0.03046, 0.033, 0.43161) * trotz(-5.4) * troty(51.5) * trotx(-13.5);

% Incorporar as transformações no robô
kukakr90kp2.base = trotx(180);  % Define a base do robô
kukakr90kp2.tool = T_8e;  % Define a ferramenta (tool) do robô

% ------------------ Configuração da Transformação entre Robôs -----------------------

% Transformação entre as bases dos robôs
T_Fab_Ftb = SE3;
T_Fab_Ftb.t = [1.73923, 0.39198, -0.46360]; % Translação
T_Fab_Ftb = T_Fab_Ftb * SE3.Rz(180);       % Rotação de 180 graus em Z

% Ângulos conhecidos da mesa posicionadora
theta_t = [pi/4, 0]; 
T_Ftb_Fde = kukaKp2.fkine(theta_t); % Cinemática direta do KP2

% Ângulos fornecidos (em radianos)
theta = [0, pi/4, 0.2438, -0.7993, 2.0019, 1.6157, -1.7229, -0.2979];

% Calcula o Jacobiano em relação à base (0)
J_base = kukakr90kp2.jacob0(theta);

% Calcula o Jacobiano em relação à ferramenta (n)
J_tool = kukakr90kp2.jacobn(theta);

% Exibir os resultados
disp('Jacobiano em relação à base (J_base):');
disp(J_base);

disp('Jacobiano em relação à ferramenta (J_tool):');
disp(J_tool);
