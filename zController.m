function [u1, zdd_cmd] = zController(z_ref, zd_ref, zdd_ref, state, kp_xyz, kd_xyz, mass)
    g = 9.81;
    phi     = getPhi(state);    % Get orientation angles
    theta   = getTheta(state);
    psi     = getPsi(state);
    
    z   = getZ(state);          % Get current pos and vel along z
    zd  = getZdot(state);
    
    kp_z = kp_xyz(3);           % Get kp and kd values along z
    kd_z = kd_xyz(3);
    
    e_z  = z_ref - z;           % Error dynamics along z
    ed_z = zd_ref - zd;

    zdd_cmd  = zdd_ref + kp_z * e_z + kd_z * ed_z;
    
    RotZYX  = rotBodytoWorld(phi, theta, psi);
    R33     = RotZYX(3,3);
    
    u1 = -mass*(zdd_cmd - g)/R33;
end