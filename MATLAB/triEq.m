function [x,y] = triEq(xi,yi,L,N)
    [x1,y1] = line2P(xi,yi,xi+L,yi,N);
    [x2,y2] = line2P(xi+L,yi,xi+0.5*L,yi+L*sind(60),N);
    [x3,y3] = line2P(xi+0.5*L,yi+L*sind(60),xi,yi,N);
    x = [x1;x2;x3];
    y = [y1;y2;y3];
end