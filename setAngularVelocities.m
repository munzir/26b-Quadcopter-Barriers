function [w1, w2, w3, w4] = setAngularVelocities(...
                                u1, pd_cmd, qd_cmd, rd_cmd, ...
                                state, I ,     kf,     km, l)
    Ix = I(1);  Iy = I(2);  Iz = I(3);
    
    p = getP(state);    q = getQ(state);    r = getR(state);
    
    F_  = u1/ kf;
    Mx_ = pd_cmd*Ix/(kf*l) + (Iz-Iy)*q*r/(kf*l);
    My_ = qd_cmd*Iy/(kf*l) + (Ix-Iz)*p*r/(kf*l);
    Mz_ = rd_cmd*Iz/km + (Iy-Ix)*p*q/km;
    
    w1 = (1/4)*(F_ + Mx_ + My_ + Mz_);
    w2 = (1/4)*(F_ - Mx_ + My_ - Mz_);
    w3 = (1/4)*(F_ - Mx_ - My_ + Mz_);
    w4 = (1/4)*(F_ + Mx_ - My_ - Mz_);
    
    w1 = -sqrt(w1); % since Tau1 (ccw) +ve
    w2 =  sqrt(w2);
    w3 = -sqrt(w3);
    w4 =  sqrt(w4);
end