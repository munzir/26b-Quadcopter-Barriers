function [p_dot, q_dot, r_dot] = getBodyVelocityDot(tau, I, p, q, r)
    tx = tau(1);    ty = tau(2);    tz = tau(3);
    Ix = I(1);      Iy = I(2);      Iz = I(3);
    
    p_ = tx - (Iz-Iy) * q * r;
    q_ = ty - (Ix-Iz) * p * r;
    r_ = tz - (Iy-Ix) * q * p;
    
    p_dot = p_/Ix;
    q_dot = q_/Iy;
    r_dot = r_/Iz;
end