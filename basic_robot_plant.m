function [xPred, u_odom] = basic_robot_plant(x, u, Ts)

x = x(1:3, 1);

c_k = cos(x(3));
s_k = sin(x(3));

xPred = zeros(3, 1);

u_odom = [u(1)*c_k*Ts; u(1)*s_k*Ts; u(2)*Ts];

xPred(1) = x(1) + u(1)*c_k*Ts;
xPred(2) = x(2) + u(1)*s_k*Ts;

xPred(3) = x(3) + u(2)*Ts;