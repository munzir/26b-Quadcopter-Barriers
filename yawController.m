function [r_cmd] = yawController(psi_ref, psi, kp_yaw)
    e_psi = psi_ref - psi;
    r_cmd = kp_yaw * e_psi;
end