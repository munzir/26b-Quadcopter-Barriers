function [Vdot] = get2x2MatrixDeriv(W, Wdot)
R11 = -W(1,2);      R11dot  = -Wdot(1,2);
R12 = -W(2,2);      R12dot  = -Wdot(2,2);
R21 = W(1,1);       R21dot  = Wdot(1,1);
R22 = W(2,1);       R22dot  = Wdot(2,1);

den = R11*R22 - R12*R21;
num = R11*R22dot - R12*R21dot - R21*R12dot + R22*R11dot;

V11dot  = R12*num/den - R12dot;
V12dot  = R11dot - R11*num/den;
V21dot  = R22*num/den - R22dot;
V22dot  = R21dot - R21*num/den;
Vdot    = [V11dot V12dot; V21dot V22dot]/den;

end