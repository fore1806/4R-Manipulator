function [H] = MTH(thetai,di,ai,alphai,angleType)
% Función para calcular la Matriz de Transformación Homogénea
% dados los parámetros DHstd en diferentes posibles sistemas
% numéricos
%
% Uso: [H] = MTH(thetai,di,ai,alphai,angleType)
%
% H             Matriz de transformación Homogénea
%
% thetai        Angulo de articulación
% di            Desplazamiento de articulación
% ai            Longitud de eslabón
% alphai        Angulo de torsión de la articulación
% angleType     0 -> radianes
%               1 -> degrees
%               2 -> theta en radianes y alpha en degrees
%               3 -> theta en degrees y alpha en radianes

    if angleType == 0 %rad
        H = [cos(thetai) -sin(thetai)*cos(alphai) sin(thetai)*sin(alphai) ai*cos(thetai);sin(thetai) cos(thetai)*cos(alphai) -cos(thetai)*sin(alphai) ai*sin(thetai); 0 sin(alphai) cos(alphai) di; 0 0 0 1];
    elseif angleType == 1 %degrees
        H = [cosd(thetai) -sind(thetai)*cosd(alphai) sind(thetai)*sind(alphai) ai*cosd(thetai);sind(thetai) cosd(thetai)*cosd(alphai) -cosd(thetai)*sind(alphai) ai*sind(thetai); 0 sind(alphai) cosd(alphai) di; 0 0 0 1];
    elseif angleType == 2 %thetai en rad, alphai en degree
        H = [cos(thetai) -sin(thetai)*cosd(alphai) sin(thetai)*sind(alphai) ai*cos(thetai);sin(thetai) cos(thetai)*cosd(alphai) -cos(thetai)*sind(alphai) ai*sin(thetai); 0 sind(alphai) cosd(alphai) di; 0 0 0 1];
    elseif angleType == 3 %thetai en degree, alphai en rad
        H = [cosd(thetai) -sind(thetai)*cos(alphai) sind(thetai)*sin(alphai) ai*cosd(thetai);sind(thetai) cosd(thetai)*cos(alphai) -cosd(thetai)*sin(alphai) ai*sind(thetai); 0 sin(alphai) cos(alphai) di; 0 0 0 1];
    end
end