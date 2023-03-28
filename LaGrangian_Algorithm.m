% Lagrangian Algorithm
% Curran Robertson
% March 21, 2023

% Max 3 Dof
% No Non-conservative forces

% Symbols
syms t q1(t) q2(t) q3(t) m1 m2 m3 k s g y

% Generalized Coordinates
q1(t) = x; % x position
q2(t) = s; % elastic stretch
q3(t) = fi; % angle

% Kinetic Energy
q1Dot = diff(q1, t);
q2Dot = diff(q2, t);
q3Dot = diff(q3, t);

T1 = (m1/2)*(q1Dot^2);
T2 = (m2/2)*(q2Dot^2);
T3 = (m3/2)*(q3Dot^2);
T = T1 + T2 + T3;

% Potential Energy
V2_e = (1/2)*k*s^2;
V3_g = m3*g*y;
V = V2_e + V3_g;

% Lagrangian Function
L = T - V

% Partial Derivatives
D1 = diff(L, q1)
D2 = diff(L, q2);
D3 = diff(L, q3);

DLDxDot = diff(L, q1Dot);
DLDfiDot = diff(L, q2Dot);
DLDsDot = diff(L, q3Dot);

% Time derivatives
DLDxDotDot = diff(DLDxDot, t);
DLDfiDotDot = diff(DLDfiDot, t);
DLDsDotDot = diff(DLDsDot, t);

% Lagrangian Equation of Motion
q1DotDot = diff(q1, t, 2);
q2DotDot = diff(q2, t, 2);
q3DotDot = diff(q3, t, 2);

eqn1 = DLDxDotDot - D1;
eqn2 = DLDfiDotDot - D2;
eqn3 = DLDsDotDot - D3;

% Solutions
q1Accel = solve(eqn1, q1DotDot)
q2Accel = solve(eqn2, q2DotDot)
q3Accel = solve(eqn3, q3DotDot)
