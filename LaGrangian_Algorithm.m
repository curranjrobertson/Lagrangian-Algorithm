% Work in Progress
% Lagrangian Algorithm
% Curran Robertson
% March 21, 2023

% Max 3 Dof
% No Non-conservative forces
clear all; clc; close all;

% Symbols
syms m1 m2 g l s fi(t) x(t) diffq1(t) diffq2(t)

q1(t) = x(t)   
q2(t) = fi(t)
y1 = 0
y2(t) = -(l+s)*cos(fi(t))

% Kinetic Energy
dq1(t) = diff(q1(t), t)
dq2(t) = diff(q2(t), t)

T1(t) = (m1/2)*(dq1(t)^2)
T2(t) = (m2/2)*(dq2(t)^2)
T = T1 + T2

% Potential Energy
V1 = m1*g*y1 % gravitational
V2 = m2*g*y2(t) % gravitational
V = V1 + V2

% Lagrangian Function
L = T - V

% Partial Derivatives
D1(t) = functionalDerivative(L, q1(t))
D2(t) = functionalDerivative(L, q2(t))

L(t) = subs(L(t),diff(x(t), t), diffq1(t))
D3(t) = functionalDerivative(L(t), diffq1(t))
L(t) = subs(L(t),diff(fi(t), t), diffq2(t))
D4(t) = functionalDerivative(L, diffq2(t))

% Time derivatives
D5(t) = diff(subs(D3(t),diffq1(t),diff(q1,t)), t)
D6(t) = diff(subs(D4(t),diffq2(t),diff(q2,t)), t)

% Lagrangian Equation of Motion
eqn = [simplify(D5 - D1); simplify(D6 - D2)] == 0
assumeAlso([m1 m2],'positive')
eqn = simplify(eqn)
