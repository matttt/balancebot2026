function f = twip_nonlinear_dynamics(y,u,p)

% States
th = y(2);
dx = y(4);
dth = y(5);
dpsi = y(6);

% Controls
uL = u(1);
uR = u(2);

a11 = p.mp + 2*p.mw + 2*p.Iw0/p.r0^2;
a12 = p.mp*p.l*cos(th);
a21 = a12;
a22 = p.Ipy + p.mp*p.l^2;
a33 = p.Ipz + 2*p.Iw0xy+(p.mw+p.Iw0/p.r0^2)*p.d^2/2 - (p.Ipz-p.Ipx-p.mp*p.l^2)*sin(th)^2;
c12 = -p.mp*p.l*dth*sin(th);
c13 = -p.mp*p.l*dpsi*sin(th);
c23 = (p.Ipz-p.Ipx-p.mp*p.l^2)*dpsi*sin(th)*cos(th);
c31 = p.mp*p.l*dpsi*sin(th);
c32 = -(p.Ipz-p.Ipx-p.mp*p.l^2)*dpsi*sin(th)*cos(th);
c33 = -(p.Ipz-p.Ipx-p.mp*p.l^2)*dth*sin(th)*cos(th);
d11 = 2*p.b/p.r0^2;
d12 = -2*p.b/p.r0;
d21 = d12;
d22 = 2*p.b;
d33 = (p.d^2/(2*p.r0^2))*p.b;

M = [a11 a12 0; a21 a22 0; 0 0 a33];
C = [0 c12 c13; 0 0 c23; c31 c32 c33];
D = [d11 d12 0; d21 d22 0; 0 0 d33];
B = [1/p.r0 1/p.r0; -1 -1; -p.d/(2*p.r0) p.d/(2*p.r0)];
G = [0 -p.mp*p.l*p.g*sin(th) 0].';

acc = M\(-(C+D)*[dx;dth;dpsi] - G + B*[-uL;-uR]);
f = [dx;dth;dpsi;acc];


end

