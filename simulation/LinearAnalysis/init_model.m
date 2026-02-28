function p = init_model()

% Body parameters
p.mp = 2;          % body mass
p.Ipx = 0.015625;  % body x inertia
p.Ipy = 0.0118625; % body y inertia
p.Ipz = 0.0118625; % body z inertia
p.d = 0.29;        % distance between wheels
p.l = 0.2907;      % pendulum length
p.b = 0.01;        % joint damping coefficient
p.g = 9.81;        % gravity 

% Wheel parameters
p.r0 = 0.07;       % wheel radius
p.mw = 0.1;        % wheel mass
p.Iw0 = 0.000735;  % wheel axis inertia
p.Iw0xy = 0.00039; % wheel off-axis inertia

% Generate partials
y = sym('y', [6,1], 'real');
u = sym('u', [2,1], 'real');
f = twip_nonlinear_dynamics(y,u,p);
A = jacobian(f,y);
B = jacobian(f,u);

matlabFunction(f,"File","f_func","Vars", {y, u});
matlabFunction(A,"File","A_func","Vars", {y, u});
matlabFunction(B,"File","B_func","Vars", {y, u});

end

