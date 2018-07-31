clc; clear;
syms R11(t) R12(t) R21(t) R22(t) R33(t) V(t) p(t) q(t) u m
W = [ R21 -R11 ; R22 -R12 ];
V = inv(W);

Vd = diff(V);
pretty(Vd)

syms t
syms phi theta psi
syms phiD thetaD psiD
syms A(t) B(t) C(t)
syms Adot Bdot Cdot

Rx_phi  = [1 0 0; 0 cos(A) -sin(A); 0 sin(A) cos(A)];
Ry_theta= [cos(B) 0 sin(B); 0 1 0; -sin(B) 0 cos(B)];
Rz_psi  = [cos(C) -sin(C) 0; sin(C) cos(C) 0; 0 0 1];

R = Rz_psi*Ry_theta*Rx_phi;

R11 = cos(C(t))*cos(B(t));
R12 = cos(C(t))*sin(A(t))*sin(B(t)) - cos(A(t))*sin(C(t));
R21 = cos(B(t))*sin(C(t));
R22 = cos(A(t))*cos(C(t)) + sin(A(t))*sin(C(t))*sin(B(t));
R33 = cos(A(t))*cos(B(t));

R11dot = diff(R11);
R12dot = diff(R12);
R21dot = diff(R21);
R22dot = diff(R22);
R33dot = diff(R33);

R11dot = subs(R11dot, 'diff(A(t), t)', Adot);
R11dot = subs(R11dot, 'diff(B(t), t)', Bdot);
R11dot = subs(R11dot, 'diff(C(t), t)', Cdot);
R11dot = subs(R11dot, 'Adot', phiD);
R11dot = subs(R11dot, 'Bdot', thetaD);
R11dot = subs(R11dot, 'Cdot', psiD);
R11dot = subs(R11dot, 'A', phi);
R11dot = subs(R11dot, 'B', theta);
R11dot = subs(R11dot, 'C', psi)

R12dot = subs(R12dot, 'diff(A(t), t)', Adot);
R12dot = subs(R12dot, 'diff(B(t), t)', Bdot);
R12dot = subs(R12dot, 'diff(C(t), t)', Cdot);
R12dot = subs(R12dot, 'Adot', phiD);
R12dot = subs(R12dot, 'Bdot', thetaD);
R12dot = subs(R12dot, 'Cdot', psiD);
R12dot = subs(R12dot, 'A', phi);
R12dot = subs(R12dot, 'B', theta);
R12dot = subs(R12dot, 'C', psi)

R21dot = subs(R21dot, 'diff(A(t), t)', Adot);
R21dot = subs(R21dot, 'diff(B(t), t)', Bdot);
R21dot = subs(R21dot, 'diff(C(t), t)', Cdot);
R21dot = subs(R21dot, 'Adot', phiD);
R21dot = subs(R21dot, 'Bdot', thetaD);
R21dot = subs(R21dot, 'Cdot', psiD);
R21dot = subs(R21dot, 'A', phi);
R21dot = subs(R21dot, 'B', theta);
R21dot = subs(R21dot, 'C', psi)

R22dot = subs(R22dot, 'diff(A(t), t)', Adot);
R22dot = subs(R22dot, 'diff(B(t), t)', Bdot);
R22dot = subs(R22dot, 'diff(C(t), t)', Cdot);
R22dot = subs(R22dot, 'Adot', phiD);
R22dot = subs(R22dot, 'Bdot', thetaD);
R22dot = subs(R22dot, 'Cdot', psiD);
R22dot = subs(R22dot, 'A', phi);
R22dot = subs(R22dot, 'B', theta);
R22dot = subs(R22dot, 'C', psi)


R33dot = subs(R33dot, 'diff(A(t), t)', Adot);
R33dot = subs(R33dot, 'diff(B(t), t)', Bdot);
R33dot = subs(R33dot, 'diff(C(t), t)', Cdot);
R33dot = subs(R33dot, 'Adot', phiD);
R33dot = subs(R33dot, 'Bdot', thetaD);
R33dot = subs(R33dot, 'Cdot', psiD);
R33dot = subs(R33dot, 'A', phi);
R33dot = subs(R33dot, 'B', theta);
R33dot = subs(R33dot, 'C', psi)


