% Parâmetros iniciais
xd = [0.04; 0; 0];  % Posição desejada do efetuador
q0 = [0; 0; 0; pi/2; 0; 0; 0; 0];  % Ângulos iniciais das juntas em radianos
qk = q0;  % Inicializando qk
Q = [qk];  % Armazenar a evolução dos ângulos

ganho = 0.4;  % Ganho do algoritmo
precisao = 1e-4;  % Critério de parada
max_iter = 100;  % Máximo de iterações

clear L; % Limpar variáveis anteriores de elos

% Parâmetros DH do sistema KP90-KP2
L(1) = Revolute('d', 0.22, 'a', 0, 'alpha', pi/2, 'offset', pi/2);          % Elo 1
L(2) = Revolute('d', 0, 'a', 1.73923, 'alpha', -pi/2, 'offset', 0);         % Elo 2
L(3) = Revolute('d', 0, 'a', -0.39198, 'alpha', pi/2, 'offset', pi/2);      % Elo 3
L(4) = Revolute('d', 0, 'a', 1.35, 'alpha', 0, 'offset', 0);                % Elo 4
L(5) = Revolute('d', 0, 'a', 0.041, 'alpha', -pi/2, 'offset', pi/2);        % Elo 5
L(6) = Revolute('d', -1.2, 'a', 0, 'alpha', pi/2, 'offset', 0);             % Elo 6
L(7) = Revolute('d', 0, 'a', 0, 'alpha', -pi/2, 'offset', 0);               % Elo 7
L(8) = Revolute('d', -0.215, 'a', 0, 'alpha', -pi, 'offset', 0);            % Elo 8

% Definir o robô
kukakr90kp2 = SerialLink(L, 'name', 'KP90-KP2');

% Transformação da ferramenta
T_8e = transl(0.03046, 0.033, 0.43161) * trotz(deg2rad(-5.4)) * ...
       troty(deg2rad(51.5)) * trotx(deg2rad(-13.5));

% Incorporar as transformações no robô
kukakr90kp2.base = trotx(pi);  % Define a base do robô (correção para radianos)
kukakr90kp2.tool = T_8e;       % Define a ferramenta (tool) do robô

% Loop para resolver a cinemática inversa iterativamente
for i = 1:max_iter
    % Calcula a pose atual (transformação homogênea)
    Tk = kukakr90kp2.fkine(qk);  % Transformação homogênea atual
    xk = Tk.t;                   % Extraindo a posição atual (x, y, z)

    % Verificando o critério de parada
    erro = norm(xd - xk);
    fprintf('Iteração %d: Erro = %.6f\n', i, erro);
    if erro < precisao
        disp('Convergência alcançada!');
        break;
    end

    % Jacobiana no espaço cartesiano
    Jk = kukakr90kp2.jacob0(qk);  % Jacobiana 6x8 no espaço cartesiano
    Jk = Jk(1:3, :);             % Extraindo as linhas de posição (3x8)

    % Atualização dos ângulos das juntas
    dqk = ganho * pinv(Jk) * (xd - xk);  % Usando pseudo-inversa para evitar singularidades
    qk = qk + dqk;  % Atualização dos ângulos

    % Armazenar a solução
    Q = [Q qk];
end

% Verificar se a convergência foi alcançada
if i == max_iter
    disp('Número máximo de iterações atingido sem convergência.');
end

% Plot da evolução
kukakr90kp2.plot(Q', 'delay', 0.5, 'trail', '*');
