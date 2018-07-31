function [state] = updateDroneState(state, I, tau, g, m, F, dt)
    phi     = getPhi(state);
    theta   = getTheta(state);
    psi     = getPsi(state);
    p = getP(state);
    q = getQ(state);
    r = getR(state);
    
    % Compute orientation angle derivaties
    [phi_dot, theta_dot, psi_dot] = getEulerDeriv(state);
    
    % Compute body rate velocity derivatives
    [p_dot, q_dot, r_dot] = getBodyVelocityDot(tau, I, p, q, r);
        
    % Compute positional double derivatives (accelerations)
    RotZYX = rotBodytoWorld(phi, theta, psi);
    [ddx, ddy, ddz] = linearAccel(RotZYX, g, m, F);
    
    % Compute position derivatives (velocities)
    x_dot = getXdot(state);
    y_dot = getYdot(state);
    z_dot = getZdot(state);

    % Compute states' derivatives
    state_dot = [ x_dot      y_dot     z_dot ...
                phi_dot  theta_dot   psi_dot  ...
                    ddx        ddy       ddz  ...
                  p_dot      q_dot    r_dot];
              
    %% include unmodeled non-linearities
    
    %%
    state = state + state_dot*dt;
end