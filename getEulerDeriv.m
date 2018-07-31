function [phi_dot, theta_dot, psi_dot] = getEulerDeriv(state)
    phi = getPhi(state);
    theta = getTheta(state);
    p = getP(state);
    q = getQ(state);
    r = getR(state);
    
    R = [1 sin(phi)*tan(theta) cos(phi)*tan(theta);
        0 cos(phi) -sin(phi);
        0 sin(phi)*sec(theta) cos(phi)*sec(theta)];
    euler_angles = R*[p q r]';
    phi_dot = euler_angles(1);
    theta_dot = euler_angles(2);
    psi_dot = euler_angles(3);
end