% Lagrangian Algorithm
% Curran Robertson
% March 21, 2023

% Symbols
syms x(t) s(t) fi(t) k

% Constants
g = 9.81;

% Generalized Coordinates
m1 = 200;
m2 = 350;
m3 = 0;
q1 = x(t);
q2 = fi(t);
q3 = s(t);

% Kinetic Energy
xDot = diff(x, t)
fiDot = diff(q2, t)
sDot = diff(q3, t)

T1 = (m1/2)*(xDot^2 + sDot^2)
T2 = (m2/2)*(fiDot^2 + fiDot^2)
T3 = (m3/2)*(sDot^2 + sDot^2)
T = T1 + T2

% Potential Energy
V1_elastic = 0;
V1_gravitational = 0;
V2_elastic = (1/2)*k*q3^2;
V2_gravitational = 0;
V3_elastic = 0;
V3_gravitational = m2*g*cos(q2);
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

dL_dx_dot = diff(L, xDot)
dL_dfi_dot = diff(L, fiDot)
dL_ds_dot = diff(L, sDot)

% Time derivatives
dL_dx_dot_dot = diff(dL_dx_dot, t);
dL_dfi_dot_dot = diff(dL_dfi_dot, t);
dL_ds_dot_dot = diff(dL_ds_dot, t);

% Lagrangian Equation of Motion
xDotDot = diff(q1, t, 2)
fiDotDot = diff(q2, t, 2)
sDotDot = diff(q3, t, 2)

eqn1 = dL_dx_dot_dot - dL_dx;
eqn2 = dL_dfi_dot_dot - dL_dfi;
eqn3 = dL_ds_dot_dot - dL_ds;
xAccel = solve(eqn1, q1_dot_dot)
angularAccel = solve(eqn2, q2_dot_dot)
springAccel = solve(eqn3, q3_dot_dot)
