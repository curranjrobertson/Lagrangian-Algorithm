% Lagrangian Algorithm
% Curran Robertson
% March 21, 2023

% Symbols
syms x s fi t k

% Constants
g = 9.81;

% Generalized Coordinates
m1 = 200;
m2 = 350;
m3 = 0;
q1 = x;
q2 = fi;
q3 = s;

% Kinetic Energy
q1_dot = diff(q1, t)
q2_dot = diff(q2, t)
q3_dot = diff(q3, t)

T1 = (m1/2)*(q1_dot^2 + q3_dot^2)
T2 = (m2/2)*(q2_dot^2 + q2_dot^2)
T3 = (m3/2)*(q3_dot^2 + q2_dot^2)
T = T1 + T2

% Potential Energy
V1_elastic = 0;
V1_gravitational = 0;
V2_elastic = (1/2)*k*s^2;
V2_gravitational = 0;
V3_elastic = 0;
V3_gravitational = m2*g*cos(fi);
V1 = V1_elastic + V1_gravitational;
V2 = V2_elastic + V2_gravitational;
V3 = V3_elastic + V3_gravitational;
V = V1 + V2 + V3

% Lagrangian Function
L = T - V

% Partial Derivatives
dL_dx = diff(L, x)
dL_dfi = diff(L, fi)
dL_ds = diff(L, s)

dL_dx_dot = diff(L, )
dL_dfi_dot = diff(L, q2_dot)
dL_ds_dot = diff(L, q3_dot)

% Time derivatives
dL_dx_dot_dot = diff(dL_dx_dot, t);
dL_dfi_dot_dot = diff(dL_dfi_dot, t);
dL_ds_dot_dot = diff(dL_ds_dot, t);

% Lagrangian Equation of Motion
q1_dot_dot = diff(q1, t, 2);
q2_dot_dot = diff(q1, t, 2);
q3_dot_dot = diff(q1, t, 2);

eqn1 = dL_dx_dot_dot - dL_dx;
eqn2 = dL_dfi_dot_dot - dL_dfi;
eqn3 = dL_ds_dot_dot - dL_ds;
xAccel = solve(eqn1, q1_dot_dot)
angularAccel = solve(eqn2, q2_dot_dot)
springAccel = solve(eqn3, q3_dot_dot)
