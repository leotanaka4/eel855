% Observação: Este é o arquivo mpd.m

q0= [pi/4  -pi/2 0 0];

[T,x] = ode45('pd',[0 10], q0); 

subplot(2,1,1)
plot(T,x(:,1))
title('q1')
grid
subplot(2,1,2)
plot(T,x(:,2))
title('q2')
grid;
