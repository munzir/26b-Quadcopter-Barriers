function [p_cmd, q_cmd] = rollPitchController(...
                            u1, ...
                            xdd_cmd, ydd_cmd, ...
                            phi, theta, psi, ...
                            kp_roll_pitch, mass)
    RotZYX = rotBodytoWorld(phi, theta, psi);

    R13_cmd = -mass*xdd_cmd/u1;
    R13_actual = RotZYX(1,3);
    ex = R13_cmd - R13_actual;
    kp_roll = kp_roll_pitch(1);
    bxd_cmd = kp_roll*ex;
    
    R23_cmd = -mass*ydd_cmd/u1;
    R23_actual = RotZYX(2,3);
    ey = R23_cmd - R23_actual;
    kp_pitch = kp_roll_pitch(2);
    byd_cmd = kp_pitch*ey;
    
    R11 = RotZYX(1,1);  R12 = RotZYX(1,2);
    R21 = RotZYX(2,1);  R22 = RotZYX(2,2);
    R33 = RotZYX(3,3);
    
    R = [R21 -R11; R22 -R12];
    
    pq_cmd = (1/R33)*R*[bxd_cmd byd_cmd]';
    p_cmd = pq_cmd(1);
    q_cmd = pq_cmd(2);
end