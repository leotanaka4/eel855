clear L;

% theta da alpha type offset
L(1) = Link([ 0 -0.2848 0 pi/2 0 0 ]);
L(2) = Link([ 0 -0.0118 0 pi/2 0 pi ]);
L(3) = Link([ 0 -0.4208 0 pi/2 0 pi ]);
L(4) = Link([ 0 -0.0128 0 pi/2 0 pi ]);
L(5) = Link([ 0 -0.3143 0 pi/2 0 pi ]);
L(6) = Link([ 0 0 0 pi/2 0 pi ]);
L(7) = Link([ 0-0.3574 0 pi 0 pi ]);

base = [1  0 0  0;
        0 -1 0  0;
        0  0 -1 0;
        0  0 0  1;];

kinova = SerialLink(L, 'name', 'Kinova', 'base', base);
q0 = deg2rad([90 15 180 -130 0 55 0]);
h = 0.01;
tmax = 30; %s
wn = 0.5;
qi = q0';
xda = [0 0]';
K = 5;
beta = [50 10];
phi = 0;
z0 = 0.4338;
f = 0.0004;
alpha = 200000;
Oc = [320 240]';
pbc = [0 -0.603]';
R = [cos(phi) sin(phi);
    sin(phi) -cos(phi);];
Kp = f/z0 * diag([alpha alpha]) * R;
xc0 = Oc - Kp * pbc;
Kstar = [Kp zeros(2); zeros(2) diag(beta)];
inactive = [1 1 0 1 0 1 0];

for t = 0:h:tmax
    J = kinova.jacob0(qi);
    Jc = J(1:2,1:end);
    Jz = J(3,1:end);
    Jp = [0 1 0 -1 0 -1 0];
    Jstar = [Jc; Jz; Jp];
    
    [xcd, xdd] = calculate_xcd_and_derivative(t, wn);
    xp = kinova.fkine(qi);
    rpy = rotm2eul(xp.R);
    xc = Kp * [xp.t(1); xp.t(2)] + xc0;
    x = [xc; xp.t(3); rpy(2)];
    xd = [xcd; 0; 0];
    err = xd - x;
    Jdag = pinv(J);
    u = Jdag*(xdd+K*err);
    u(logical(inactive)) = 0;
    qk = qi + h * u;
    qi = qk;
    xda = xd;
end

function [xcd, d_xcd] = calculate_xcd_and_derivative(t, wn)
    % Função para calcular xcd(t) e sua derivada d_xcd(t)
    % t: vetor de tempo
    % wn: frequência angular natural
    
    % Componentes de xcd(t)
    x1 = 360 - 30*cos(wn*t) - 30*cos(1.5*wn*t) - 10*cos(2*wn*t);
    x2 = 150 - 20*cos(wn*t + 1.6) - 20*cos(1.5*wn*t + 1.6) - 10*cos(2*wn*t + 1.6);
    
    xcd = [x1; x2];
    
    % Derivada de xcd(t)
    dx1 = wn*30*sin(wn*t) + 1.5*wn*30*sin(1.5*wn*t) + 2*wn*10*sin(2*wn*t);
    dx2 = wn*20*sin(wn*t + 1.6) + 1.5*wn*20*sin(1.5*wn*t + 1.6) + 2*wn*10*sin(2*wn*t + 1.6);
    
    d_xcd = [dx1; dx2];
end