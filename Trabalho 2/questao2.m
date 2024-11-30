clear all;

% Parâmetros do manipulador Kinova Gen3
L(1) = Revolute('d', -0.2848, 'a', 0, 'alpha', pi/2, 'offset', 0);
L(2) = Revolute('d', -0.0118, 'a', 0, 'alpha', pi/2, 'offset', pi);
L(3) = Revolute('d', -0.4208, 'a', 0, 'alpha', pi/2, 'offset', pi);
L(4) = Revolute('d', -0.0128, 'a', 0, 'alpha', pi/2, 'offset', pi);
L(5) = Revolute('d', -0.3143, 'a', 0, 'alpha', pi/2, 'offset', pi);
L(6) = Revolute('d', 0,       'a', 0, 'alpha', pi/2, 'offset', pi);
L(7) = Revolute('d', -0.3574, 'a', 0, 'alpha', pi,   'offset', pi);

% Criação do manipulador
kinova = SerialLink(L, 'name', 'Kinova');

% Parâmetros da câmera
f = 4e-3; % Distância focal em metros
z0 = 0.4338; % Distância da câmera ao plano de trabalho
alpha = 200000; % Fator de escala em pixels/m
Oc = [320; 240]; % Origem do plano de imagem
Pbc = [0; -0.603]; % Posição da câmera em relação à base
Rbc = eye(2); % Orientação da câmera
Kp = f/z0 * diag([alpha alpha]) * Rbc;

% Configuração inicial do manipulador
q0 = deg2rad([90, 15, 180, -130, 0, 55, 0]);
qi = q0;

% Condições da simulação
h = 0.01; % Passo de tempo
tmax = 10; % Tempo máximo
t_vec = 0:h:tmax;

% Parâmetros do controlador
K = 10; % Ganho do controlador
wmax = 1; % Velocidade máxima das juntas (rad/s)

% Trajetória de referência no plano da imagem
wn = 0.5; % Frequência natural
xcd_func = @(t) [360 - 30*cos(wn*t) - 30*cos(1.5*wn*t) - 10*cos(2*wn*t); ...
                 150 - 20*cos(wn*t + 1.6) - 20*cos(1.5*wn*t + 1.6) - 10*cos(2*wn*t + 1.6)];

% Variáveis para armazenar dados
x_ref = zeros(2, length(t_vec));
x_out = zeros(2, length(t_vec));
xd = [0 0]';

% Simulação
for k = 1:length(t_vec)
    t = t_vec(k);
    
    % Trajetória desejada no plano da imagem
    xcd = xcd_func(t);
    x_ref(:, k) = xcd;

    % Cinemática direta para obter posição do efetuador
    T = kinova.fkine(qi);
    Pe = T.t; % Posição do efetuador na base do robô
    
    % Posição no plano da imagem
    xi = Kp * [Pe(1) Pe(2)]' + Oc
    x_out(:, k) = xi;

    % Erro de posição no plano da imagem
    err = xcd - xi;

    % Jacobiano geométrico
    J = kinova.jacob0(qi); % Jacobiano na base
    % Jacobiano original
    Jp = J(1:2, 1:end); 

    % Definir as colunas 3, 5 e 7 como zeros
    Jp(:, [3, 5, 7]) = 0; 
    
    % Controle cinemático
    xdd = (xd - xi) / h;
    u = pinv(Jp) * (xdd + K * err); % Cálculo das velocidades
    u = max(min(u, wmax), -wmax); % Limitação de velocidades

    % Integração (Euler)
    qi = qi + h * u';
end

% Plotagem dos resultados
figure;
subplot(2, 1, 1);
plot(t_vec, x_ref(1, :), 'r', 'LineWidth', 1.5); hold on;
plot(t_vec, x_out(1, :), 'b--', 'LineWidth', 1.5);
xlabel('Tempo (s)');
ylabel('Pixels');
legend('Referência', 'Saída');
title('Trajetória no eixo X (pixels)');
grid on;

subplot(2, 1, 2);
plot(t_vec, x_ref(2, :), 'r', 'LineWidth', 1.5); hold on;
plot(t_vec, x_out(2, :), 'b--', 'LineWidth', 1.5);
xlabel('Tempo (s)');
ylabel('Pixels');
legend('Referência', 'Saída');
title('Trajetória no eixo Y (pixels)');
grid on;
