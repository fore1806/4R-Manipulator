function [q1, q2, q3, q4] = inverseKinematics(xT, yT, zT, q4, q4T)
% Calcula la cinematica inversa del manipulador 4R basado en la posición y
% orientacion de la herramienta.
%
% Uso: [q1, q2, q3, q4] = inverseKinematics(xT, yT, zT, q4, q4T)
%
% q1        Angulo de la articulacion 1 del manipulador [°]
% q2        Angulo de la articulacion 2 del manipulador [°]
% q3        Angulo de la articulacion 3 del manipulador [°]
% q4        Angulo de la articulacion 4 del manipulador [°]
%
% xT        Posicion de la herramienta a lo largo del eje X [cm]
% yT        Posicion de la herramienta a lo largo del eje Y [cm]
% zT        Posicion de la herramienta a lo largo del eje Z [cm]
% q4        Orientacion angular de la herramienta
% q4T       Unidad del parametro q4, 0 para radianes; otro valor para
% grados

%Parámetros del Robot
e = 50e-03;
l2 = 179.81e-3;
l3 = 125.3264e-3; %m
l4 = 63.75e-03;
z0 = 123.85e-03;

%Coseno y seno del ángulo q4 dependiendo del tipo
if (q4T == 0) %q4 en radianes
    cq4 = cos(q4);
    sq4 = sin(q4);
    q4 = q4*(180/pi);    
else %q4 en grados hexadecimales
    cq4 = cosd(q4);
    sq4 = sind(q4);
end

%Determinación q1
theta = atan2(yT,xT);
d = sqrt(xT^2 + yT^2);
delta = atan2(l4*cq4,e);
d2 = sqrt(e^2+l4^2*cq4^2);
sigma = pi/2 + delta;
psi = asin((sin(sigma)*d2)/d);
q1 = theta-psi;

%Determinación de la posición de la muñeca
zW = zT-l4*sq4;
xW = xT - l4*cq4*cos(q1) + e*sin(q1);
yW = yT - l4*cq4*sin(q1) - e*cos(q1);

%Determinación de los ángulos q2 y q3
beta = atan2(zW-z0, sqrt(xW^2+yW^2));
r = sqrt(xW^2 + yW^2 + (zW-z0)^2);
gamma = acos((l2^2+r^2-l3^2)/2*r*l2);
phi = acos((l2^2+l3^2-r^2)/2*l2*l3);
q2 = beta + gamma;
q3 = -(pi - phi);

%Se convierten los ángulos a grados hexadecimales
q1 = q1*(180/pi);
q2 = q2*(180/pi);
q3 = q3*(180/pi);

end