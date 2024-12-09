
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Sistema de Controle por PD               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Observação: Este é o arquivo pd.m

function [xdot] = pd(t, x)
% A função acima recebe dois vetores:
%			- vetor t: contém o tempo inicial e o tempo final.
%			- vetor q: contém os ângulos das juntas.

% Dados do manipulador planar de dois lin

% O comprimento dos links é dado por:
a = [1;1];

% Dados do exemplo 4.2 do livro texto:
l1 = 0.5;
l2 = 0.5;
Ml1 = 50;
Ml2 = 50;  
Il1 = 10;
Il2 = 10;
Kr1 = 100;
Kr2 = 100;
Mm1 = 5;
Mm2 = 5;
Im1 = 0.01;
Im2 = 0.01;
g = 9.8; 

% Especificações da trajetória desejada:
qd1 = 0;
qd2 = 0;
qd = [qd1; qd2];
dt1 = 0;
dt2 = 0;
qddot = [dt1;dt2];

% Parâmetros do ganho:
kd=10000*eye(2); 
kp=10000*eye(2); 

% Cálculo dos parâmetros do modelo do manipulador da página 139:

% A Matriz B é:
b11 = Il1+Ml1*l1^2+(Kr1^2)*Im1+Il2+Ml2*(a(1)^2+l2^2+2*a(1)*l2*cos(x(2)))+Im2+Mm2*a(1)^2;
b12 = Il2 + Ml2*(l2^2 +a(1)*l2*cos(x(2))) + Kr2*Im2;
b22 = Il2 + Ml2^2 + Kr2^2*Im2;
b = [b11 b12; b12 b22];

% A Matriz C é:
h = -Ml2*a(1)*l2*sin(x(2));
c11 = h*x(4);
c12 = h*(x(3) + x(4));
c21 = -h*x(3);
c22 = 0;
c = [c11 c12;c21 c22];

% A Matriz G é:
g1 = (Ml1*l1 +Mm2*a(1) + Ml2*a(1))*g*cos(x(1)) + Ml2*l2*g*cos(x(1) + x(2));
g2 = Ml2*l2*g*cos(x(1) + x(2));
g = [g1;g2];

% A Matriz de estados resultante é:
q    = x(1:2); % q1 e q2 calculados pelo ODE45
qdot = x(3:4); % w1 e w2 calculados pelo ODE45

% O modelo para este controle é:  VOCES CALCULAM
%tau = 0.0;

erro = [qd-q ; qddot-qdot]
%errod = [errod erro];
k=[kp kd];
tau=k*erro + g

binv = inv(b);

xdot =[ qdot ; -binv*(c*qdot + g - tau)];

