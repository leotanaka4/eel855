clear all; close all;

% Definicao dos parametros DH com juntas revolutas
L(1) = Revolute('d', -0.2848, 'a', 0, 'alpha', pi/2, 'offset', 0);
L(2) = Revolute('d', -0.0118, 'a', 0, 'alpha', pi/2, 'offset', pi);
L(3) = Revolute('d', -0.4208, 'a', 0, 'alpha', pi/2, 'offset', pi);
L(4) = Revolute('d', -0.0128, 'a', 0, 'alpha', pi/2, 'offset', pi);
L(5) = Revolute('d', -0.3143, 'a', 0, 'alpha', pi/2, 'offset', pi);
L(6) = Revolute('d', 0,       'a', 0, 'alpha', pi/2, 'offset', pi);
L(7) = Revolute('d', -0.3574, 'a', 0, 'alpha', pi,   'offset', pi);

% Criacao do manipulador
kinova_gen3 = SerialLink(L, 'name', 'Kinova');
kinova_gen3.base = trotx(180);  % Define a base do robo

% Parametros do sistema
f = 4e-3; % Distancia focal (m)
z0 = 0.4338; % Profundidade (m)
alpha = 200000; % Fator de escala (pixels/m)
Oc = [320; 240]; % Origem do plano da imagem (pixels)
Pbc = [0;-0.603; 0]; % Posicao da camera em relacao a base (m)
phi = 0; % Angulo de orientacao da camera (rad)
wn = 0.5; % Frequencia angular (rad/s)

% Ganho do controlador
K = 1; % Ajuste o ganho conforme necessario

% Trajetoria desejada no plano da imagem
h = 0.01; % Passo de tempo
tmax = 25.2; % Tempo maximo (s)
t = 0:h:tmax; % Tempo de simulacao (s)
xcd = [
    360- 30*cos(wn*t)- 30*cos(1.5*wn*t)- 10*cos(2*wn*t);
    150- 20*cos(wn*t + 1.6)- 20*cos(1.5*wn*t + 1.6)- 10*cos(2*wn*t + 1.6)
];
xcdd = [
    30*sin(wn*t)*wn + 30*sin(1.5*wn*t)*1.5*wn + 10*sin(2*wn*t)*2*wn;
    20*sin(wn*t + 1.6)*wn + 20*sin(1.5*wn*t + 1.6)*1.5*wn + 10*sin(2*wn*t + 1.6)*2*wn
];

% Transformacao camera/espaco de trabalho
Kp = (f / z0) * diag([alpha,-alpha]) * [cos(phi), sin(phi); sin(phi),-cos(phi)];
xc0 = (f / z0) * diag([alpha,-alpha]) * [-cos(phi),-sin(phi);-sin(phi), cos(phi)]*Pbc(1:2) + Oc;

% Inicializacao
xc = zeros(2, length(t)-1); % Posicao no plano da imagem
theta = deg2rad([90; 15; 180;-130; 0; 55; 0]); % Angulos iniciais das juntas (rad)
theta_log = zeros(length(theta), length(t)-1); % Log para salvar os valores de theta
xyz = zeros(3, length(t)-1); % Posicao no espaco do efetuador

% Erros
ex = zeros(1, length(t)-1); % Erro X
ey = zeros(1, length(t)-1); % Erro Y
u_log = zeros(4, length(t)-1); % Log para salvar os valores de u

% Simulacao
for k = 1:length(t)-1
    % Cinematica direta (posicao no espaco de trabalho)
    x = kinova_gen3.fkine(theta).t; % Posicao do efetuador
    xyz(:, k) = x; % Salvar posicao do efetuador no espaco
    % Projecao no plano da imagem
    xc(:, k) = Kp * x(1:2) + xc0;
    
    % Erro na imagem
    ecd = xcd(:, k)- xc(:, k);
    ex(k) = ecd(1); % Erro no eixo X
    ey(k) = ecd(2); % Erro no eixo Y
    
    % Jacobiano do manipulador (somente juntas controladas)
    J = kinova_gen3.jacob0(theta); % Jacobiano completo
    J_reduced = J(1:2, [1, 2, 4, 6]); % Considerar apenas as colunas de juntas 1, 2, 4 e 6
    
    % Controle cinemático
    Jv = Kp * J_reduced;
    xcd_dot = xcdd(:, k); % Derivada da trajetoria desejada
    u = pinv(Jv) * (xcd_dot + K * ecd);
    
    % Verificar se algum valor de u é maior que 1
    if any(abs(u) > 1)
        disp('Alerta: Um ou mais valores de u excedem 1.');
        disp('Valores de u:');
        disp(u);
    end
    
    % Atualizacao dos angulos das juntas
    theta_log(:, k) = theta; % Salvar angulos
    theta([1, 2, 4, 6]) = theta([1, 2, 4, 6]) + u * h;
    
    % Log de u
    u_log(:, k) = u;
end

% Graficos
figure;
subplot(2, 1, 1);
plot(t(1:end-1), xcd(1, 1:end-1), 'r', t(1:end-1), xc(1, :), 'b--');
xlabel('Tempo (s)'); ylabel('Pixel X');
legend('Trajetoria de referencia', 'Trajetoria do efetuador');
grid on;

subplot(2, 1, 2);
plot(t(1:end-1), xcd(2, 1:end-1), 'r', t(1:end-1), xc(2, :), 'b--');
xlabel('Tempo (s)'); ylabel('Pixel Y');
legend('Trajetoria de referencia', 'Trajetoria do efetuador');
grid on;

figure; % Grafico XY (trajetoria no plano da tela)
plot(xcd(1, :), xcd(2, :), 'r', 'LineWidth', 1.5); hold on;
plot(xc(1, :), xc(2, :), 'b--', 'LineWidth', 1.5);
xlabel('Pixel X'); ylabel('Pixel Y');
legend('Trajetoria de referencia', 'Trajetoria do efetuador');
grid on;

xdr = pinv(Kp)*(xcd-xc0);
zdr = z0*ones(size(xcd(1,:)));
figure; % Grafico XYZ (trajetoria no espaco do efetuador)
plot3(xdr(1, :), xdr(2, :), zdr, 'r-', 'LineWidth', 1.5); hold on;
plot3(xyz(1, :), xyz(2, :), xyz(3, :), 'b--', 'LineWidth', 1.5);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
grid on; axis equal;

% Grafico de erro
figure;
subplot(2, 1, 1);
plot(t(1:end-1), ex, 'r');
xlabel('Tempo (s)'); ylabel('Erro X (pixels)');
grid on;

subplot(2, 1, 2);
plot(t(1:end-1), ey, 'r');
xlabel('Tempo (s)'); ylabel('Erro Y (pixels)');
grid on;

% Grafico de u para as juntas
figure;
subplot(4, 1, 1);
plot(t(1:end-1), u_log(1, :), 'b');
xlabel('Tempo (s)'); ylabel(['u Junta ' num2str(1)]);
subplot(4, 1, 2);
plot(t(1:end-1), u_log(2, :), 'b');
xlabel('Tempo (s)'); ylabel(['u Junta ' num2str(2)]);
subplot(4, 1, 3);
plot(t(1:end-1), u_log(3, :), 'b');
xlabel('Tempo (s)'); ylabel(['u Junta ' num2str(4)]);
grid on;
subplot(4, 1, 4);
plot(t(1:end-1), u_log(4, :), 'b');
xlabel('Tempo (s)'); ylabel(['u Junta ' num2str(7)]);
grid on;
