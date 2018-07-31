function [R11dot, R12dot, R21dot, R22dot, R33dot] = getRotDerivative(state)

[phiDot, thetaDot, psiDot] = getEulerDeriv(state);
phi = getPhi(state);    theta = getTheta(state);        psi = getPsi(state);

cosTheta = cos(theta);    sinTheta = sin(theta);
cosPhi = cos(phi);        sinPhi = sin(phi);
cosPsi = cos(psi);        sinPsi = sin(psi);

R11dot = -sinTheta*thetaDot*cosPsi - cosTheta*sinPsi*psiDot;
R12dot = cosPhi*phiDot*sinTheta*cosPsi + cosTheta*thetaDot*sinPhi*cosPsi - ...
    sinPsi*psiDot*sinPhi*sinTheta + sinPhi*phiDot*sinPsi - cosPsi*psiDot*cosPhi;

R21dot = -sinTheta*thetaDot*sinPsi + cosPsi*psiDot*cosTheta;
R22dot = cosPhi*phiDot*sinTheta*sinPsi + cosTheta*thetaDot*sinPhi*sinPsi + ...
    cosPsi*psiDot*sinPhi*sinTheta - sinPhi*phiDot*cosPsi - sinPsi*psiDot*cosPhi;

R33dot = -sinPhi*phiDot*cosTheta - sinTheta*thetaDot*cosPhi;

end