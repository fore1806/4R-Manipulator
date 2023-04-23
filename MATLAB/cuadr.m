function [x,y] = cuadr(xi,yi,L,N)
    a = L*cosd(45);
    [x1,y1] = line2P(xi,yi,xi-a,yi+a,N);
    [x2,y2] = line2P(xi-a,yi+a,xi-2*a,yi,N);
    [x3,y3] = line2P(xi-2*a,yi,xi-a,yi-a,N);
    [x4,y4] = line2P(xi-a,yi-a,xi,yi,N);
    x = [x1;x2;x3;x4];
    y = [y1;y2;y3;y4];
end