% Configuração dos Waypoints
waypoints = [
    0.04,  0.00, 0.0;
   -0.04,  0.00, 0.0;
   -0.04,  0.01, 0.0;
    0.04,  0.01, 0.0;
    0.04,  0.02, 0.0;
   -0.04,  0.02, 0.0;
   -0.04,  0.03, 0.0;
    0.04,  0.03, 0.0;
    0.04,  0.04, 0.0;
   -0.04,  0.04, 0.0;
];

% ------------------ Configuração do Robô Kuka KR90 ------------------------

% Definição dos elos
L(1) = Revolute('d', -0.675, 'a', 0.35, 'alpha', pi/2, 'offset', 0);      
L(2) = Revolute('d', 0, 'a', 1.35, 'alpha', 0, 'offset', 0);              
L(3) = Revolute('d', 0, 'a', 0.041, 'alpha', -pi/2, 'offset', pi/2);      
L(4) = Revolute('d', -1.2, 'a', 0, 'alpha', pi/2, 'offset', 0);           
L(5) = Revolute('d', 0, 'a', 0, 'alpha', -pi/2, 'offset', 0);             
L(6) = Revolute('d', -0.215, 'a', 0, 'alpha', -pi, 'offset', 0);          

% Transformações da base e ferramenta
T_base_kr90 = trotx(180); % Rotação de 180 graus no eixo X
T_tool_kr90 = transl(0.03046, 0.033, 0.43161) * trotz(-5.4) * troty(51.5) * trotx(-13.5);

% Criação do robô Kuka KR90
kukaKr90 = SerialLink(L, 'name', 'Kr90', 'base', T_base_kr90, 'tool', T_tool_kr90);

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

% ------------------ Configuração da Transformação entre Robôs -----------------------

% Transformação entre as bases dos robôs
T_Fab_Ftb = SE3;
T_Fab_Ftb.t = [1.73923, 0.39198, -0.46360]; % Translação
T_Fab_Ftb = T_Fab_Ftb * SE3.Rz(180);       % Rotação de 180 graus em Z

% Ângulos conhecidos da mesa posicionadora
theta_t = [pi/4, 0]; 
T_Ftb_Fde = kukaKp2.fkine(theta_t); % Cinemática direta do KP2

% ------------------ Cálculo da Cinemática Inversa e Trajetória -----------------------

% Configurações iniciais
q0 = zeros(1, kukaKr90.n); % Ângulos iniciais das juntas
tol = 1e-6;                % Tolerância para solução da cinemática inversa
lambda = 0.1;              % Taxa de regularização

% Matriz para armazenar as soluções das juntas
q_solutions = zeros(size(waypoints, 1), kukaKr90.n);

% Iteração para cada waypoint
for i = 1:size(waypoints, 1)
    % Define a transformação desejada para o waypoint atual
    T_Fde_Ft = SE3.Rz(90) * SE3.Rx(180); % Orientação fixa
    T_Fde_Ft.t = waypoints(i, :);        % Posição do waypoint
    
    % Transformação total em relação à base absoluta
    T_Fab_Ft = T_Fab_Ftb * T_Ftb_Fde * T_Fde_Ft;
    
    % Calcula a cinemática inversa
    q_solutions(i, :) = kukaKr90.ikine(T_Fab_Ft, q0, 'tol', tol, 'lambda', lambda, 'mask', [1 1 1 1 1 1]);
    
    % Valida a solução obtida com a cinemática direta
    T_check = kukaKr90.fkine(q_solutions(i, :));
    disp(['Waypoint ', num2str(i)]);
    disp('Transformação homogênea obtida:');
    disp(robot.fkine([0, pi/4, q_solutions(i,:)]).T);
    disp('Angulos das juntas:');
    disp(q_solutions(i, :));
end

% ------------------ Plotagem da Trajetória -----------------------

figure; hold on;
kukaKr90.plot(q_solutions, 'workspace', [-6 6 -6 6 -6 6]);
T_base_kp2_adjusted = T_Fab_Ftb.T * transl(0, 0, 0.865) * trotz(-90) * trotx(90);
kukaKp2.base = T_base_kp2_adjusted;
kukaKp2.plot(theta_t);
title('Trajetória em Zigue-Zague');