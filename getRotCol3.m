function [R13, R23, R33, R] = getRotCol3(droneState)
    phi = getPhi(droneState);
    theta = getTheta(droneState);
    psi = getPsi(droneState);

    R = rotBodytoWorld(phi, theta, psi);
    R13 = R(1,3);
    R23 = R(2,3);
    R33 = R(3,3);
end