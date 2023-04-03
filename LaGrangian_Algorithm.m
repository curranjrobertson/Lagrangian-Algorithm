% Lagrangian Algorithm
% Curran Robertson
% March 21, 2023

% Max 3 Dof
% No Non-conservative forces
clear all; clc; close all;

% Symbols
syms q1(t) q2(t) dq1(t) dq2(t) y1 y2 m1 m2 g

% Kinetic Energy
% dq1(t) = diff(q1(t), t)
% dq2(t) = diff(q2(t), t)

T1(t) = (m1/2)*(dq1(t)^2)
T2(t) = (m2/2)*(dq2(t)^2)
T = T1 + T2

% Potential Energy
V1 = m1*g*y1 % gravitational
V2 = m2*g*y2 % gravitational
V = V1 + V2

% Lagrangian Function
L = T - V

% Partial Derivatives
D1(t) = functionalDerivative(L, q1(t))
D2(t) = functionalDerivative(L, q2(t))

D3(t) = functionalDerivative(L, dq1(t))
D4(t) = functionalDerivative(L, dq2(t))

% Time derivatives
% Time derivatives
D5(t) = diff(subs(D3(t),dq1,diff(q1,t)), t)
D6(t) = diff(subs(D4(t),dq2,diff(q2,t)), t)

% Lagrangian Equation of Motion
eqn = [simplify(D5 - D1); simplify(D6 - D2)] == 0
assumeAlso([m1 m2],'positive')
eqn = simplify(eqn)

% D5(t) = functionalDerivative(D3(t), t)
% D6(t) = functionalDerivative(D4(t), t)
% 
% % Lagrangian Equation of Motion
% accel1 = functionalDerivative(q1, t, 2)
% accel2 = functionalDerivative(q2, t, 2)

eqn1 = simplify(D5 - D1);
eqn2 = simplify(D6 - D2);

% % Solutions
% q1Accel = solve(eqn1, accel1);
% q2Accel = solve(eqn2, accel2);
