function [R13dot, R23dot] = getRotCol3Deriv(R, p, q)
R11 = R(1,1);   R12 = R(1,2);
R21 = R(2,1);   R22 = R(2,2);
R33 = R(3,3);

W = [R21 -R11; R22 -R12];
Rvalues = R33*pinv(W)*[p q]';
R13dot = Rvalues(1);
R23dot = Rvalues(2);
end