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
T_Fab_Ftb = SE3; % Inicializa uma transformação homogênea (SE3)
T_Fab_Ftb.t = [1.73923, 0.39198, -0.46360]; % Define a translação
T_Fab_Ftb = T_Fab_Ftb * SE3.Rz(180); % Aplica uma rotação de 180 graus em torno do eixo Z

% Ângulos conhecidos da mesa posicionadora (KP2)
theta_t = [pi/4, 0]; 
T_Ftb_Fde = kukaKp2.fkine(theta_t); % Calcula a cinemática direta da mesa posicionadora (KP2)

% Ângulos fornecidos para o robô KUKA KR90 (em radianos)
theta = [0, pi/4, 0.2438, -0.7993, 2.0019, 1.6157, -1.7229, -0.2979];

% Calcula o Jacobiano do robô KUKA KR90 em relação à ferramenta
J_tool = kukakr90kp2.jacobe(theta);

% Calcula o Jacobiano do robô KUKA KR90 em relação à base
J_base = kukakr90kp2.jacob0(theta);

disp('Jacobiano em relação à ferramenta (J_tool):');
disp(J_tool);

disp('Jacobiano em relação à base (J_base):');
disp(J_base);

% Matriz de permutação P para ajustar a ordem de colunas
P = [
    0 1;
    1 0
];

% Vetor p_ne (vetor posição da ferramenta no espaço cartesiano)
p_dt = [0.04; 0; 0];

% Matriz identidade 3x3
I = eye(3);

% Matriz anti-simétrica associada ao vetor p_dt
p_dt_cross = [
    0     -p_dt(3)  p_dt(2);
    p_dt(3)  0     -p_dt(1);
   -p_dt(2)  p_dt(1)  0
];

% Construção da matriz adjunta (adj_matrix)
adj_matrix = [
    I,        -p_dt_cross; % Parte superior: identidade e anti-simétrica
    zeros(3), I            % Parte inferior: identidade
];

% Jacobiano do robô KUKA KR90 em relação à ferramenta (considerando apenas seus ângulos)
J_kukakr90_tool = kukakr90.jacobe(theta(3:8));

% Jacobiano da mesa posicionadora (KP2) em relação à base
J_kukaKp2_base = kukaKp2.jacobe(theta_t);

% Jacobiano transformado para o sistema T_Fde_Ft
J_T_Fde_Ft = tr2jac(inv(T_Ftb_Fde.T) * inv(T_Fab_Ftb.T) * kukakr90.fkine(theta(3:8)).T);

% Jacobiano transformado para o sistema T_Ft_Fde
J_T_Ft_Fde = tr2jac(inv(kukakr90.fkine(theta(3:8)).T)*T_Fab_Ftb.T*T_Ftb_Fde.T);

% Concatenação final do Jacobiano ajustado
J_tool = [J_T_Fde_Ft * adj_matrix * -J_kukaKp2_base * P, J_kukakr90_tool];

% Exibe o Jacobiano em relação à ferramenta
disp('Jacobiano em relação à ferramenta (J_tool):');
disp(J_tool);

% Concatenação final do jacobiano ajustado
J_base = [adj_matrix * -J_kukaKp2_base * P, J_T_Ft_Fde * J_kukakr90_tool];

% Exibe o Jacobiano em relação à base
disp('Jacobiano em relação à base (J_base):');
disp(J_base);