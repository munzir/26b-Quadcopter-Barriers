function [xdd_cmd, ydd_cmd] = xyController(x_info, y_info, state, kp_xyz, kd_xyz)
    x_ref     = x_info(1);
    xd_ref    = x_info(2);
    xdd_ref   = x_info(3);        % feed forward
    x_actual  = getX(state);
    xd_actual = getXdot(state);
    
    e_x   = x_ref - x_actual;     % error
    ed_x  = xd_ref - xd_actual;   % error derivative
    kp_x  = kp_xyz(1);           	% get kp and kd for x
    kd_x  = kd_xyz(1);
    xdd_cmd = kp_x*e_x + kd_x*ed_x + xdd_ref;
    
    y_ref     = y_info(1);
    yd_ref    = y_info(2);
    ydd_ref   = y_info(3);        % feed forward
    y_actual  = getY(state);
    yd_actual = getYdot(state);
    
    e_y   = y_ref - y_actual;     % error
    ed_y  = yd_ref - yd_actual;   % error derivative
    kp_y  = kp_xyz(2);               % get kp and kd for y
    kd_y  = kd_xyz(2);
    ydd_cmd = kp_y*e_y + kd_y*ed_y + ydd_ref;
end