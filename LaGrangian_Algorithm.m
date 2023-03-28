% Lagrangian Algorithm
% Curran Robertson
% March 21, 2023

% Symbols
syms k1

% Constants
g = 9.81;


% Generalized Coordinates
m1 = 1;
m2 = 2;
m3 = 3;
q1 = 
q2 = 
q3 = 

% Kinetic Energy
q1_dot = diff(q1, t);
q2_dot = diff(q2, t);
q3_dot = diff(q3, t);

T1 = (m1/2)*(q1_dot^2 + q3_dot^2);
T2 = (m2/2)*(q2_dot^2 + q2_dot^2);
T3 = (m3/2)*(q3_dot^2 + q2_dot^2);
T = T1 + T2 + T3;

% Potential Energy
V1_e = (1/2)*k1*s1^2;
V1_g = m*g*y1;
V2_e = (1/2)*k2*s2^2;
V2_g = m2*g*y2;
V3_e = (1/2)*k3*s3^2;
V3_g = m3*g*y3;
V1 = V1_e + V1_g;
V2 = V2_e + V2_g;
V3 = V3_e + V3_g;
V = V1 + V2 + V3;

% Lagrangian Function
L = T - V;

% Partial Derivatives
dL_dx = diff(L, x);
dL_dfi = diff(L, fi);
dL_ds = diff(L, s);

% Partial Derivatives
dL_dx_dot = diff(L, x_dot);
dL_dfi_dot = diff(L, fi_dot);
dL_ds_dot = diff(L, s_dot);

% Time derivatives
dL_dx_dot_dot = diff(dL_dx_dot, t);
dL_dfi_dot_dot = diff(dL_dfi_dot, t);
dL_ds_dot_dot = diff(dL_ds_dot, t);

% Lagrangian Equation of Motion
eqn1 = dL_dx_dot_dot - dL_dx;
eqn2 = dL_dfi_dot_dot - dL_dfi;
eqn3 = dL_ds_dot_dot - dL_ds;
solve(eqn1, x_dot_dot);
solve(eqn2, fi_dot_dot);
solve(eqn3, s_dot_dot);
