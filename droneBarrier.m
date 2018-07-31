clc; clear;
% close all;

plot_flag   = 1;
use_QP_u1   = 1;
use_QP_txy  = 1;

%% Reference Trajectory (Flight plan)
tic
fprintf('Setting up flight trajectory ... ');
total_time = 20; % 20 seconds
dt = 0.01;
time = linspace(0, total_time, floor(total_time/dt) );

amp = 2;
omega_x = 0.8; omega_y = 0.4; omega_z = 0.4;

x_arg   = omega_x*time;
x_ref   = amp*sin(x_arg);
xd_ref  = omega_x*amp*cos(x_arg);
xdd_ref = -omega_x*omega_x*amp*sin(x_arg);

y_arg   = omega_y*time;
y_ref   = amp*cos(y_arg);
yd_ref  = -omega_y*amp*sin(y_arg);
ydd_ref = -omega_y*omega_y*amp*cos(y_arg);

z_arg   = omega_z*time;
z_ref   = amp*cos(z_arg);
zd_ref  = -omega_z*amp*sin(z_arg);
zdd_ref = -omega_z*omega_z*amp*cos(z_arg);

psi_ref = atan2(yd_ref, xd_ref);
refTrajectory = [x_ref' y_ref' z_ref' xd_ref' yd_ref' zd_ref' xdd_ref' ydd_ref' zdd_ref' psi_ref'];

save 'refTrajectory' 'refTrajectory'

fprintf('Done.         ');
toc

%% Drone characteristics
tic
fprintf('Setting up drone & PID parameters ... ');
kf = 1.0;           % constant relating force and rotor speed
km = 1.0;           % constant relating torque and rotor speed
m  = 1.0;           % mass of quad in kg
g  = 9.81;          % gravity
L  = 0.25;          % rotor-to-rotor distance
l  = L/(2*sqrt(2));
Ix = 0.1;           % inertia along x-axis
Iy = 0.1;           % inertia along y-axis
Iz = 0.2;           % inertia along z-axis

%% PID parameters for each controller
% x, y, z kp and kd values
kp_xyz = [12.0, 12.0, 4.0];   % x, y, ,z kp values
kd_xyz = [8.0, 8.0, 2.0];   % x, y, z, kd values (2nd order)

kp_roll_pitch = [8.0, 8.0]; % roll, pitch kp values (1st order)
kp_yaw = 8.0;

kp_pqr = [20.0 20.0 20.0];  % p, q, r kp values (1st order)

droneState = [ x_ref(1)   y_ref(1)     z_ref(1) ...
    0.0       0.0    psi_ref(1) ...
    xd_ref(1)  yd_ref(1)    zd_ref(1) ...
    0.0       0.0          0.0] ;

fprintf('Done.    ');
toc

%% Barrier Parameters
px = 2;     % ellipse pos axis along x-axis
py = 2;     % ellipse pos axis along y-axis
pz = 2;   % ellipse pos axis along z-axis

vx = 1.5;   % ellipse vel axis along x
vy = 0.7;   % ellipse vel axis along y-axis
vz = 0.5;   % ellipse vel axis along z-axis

cz  = 2;     % elliposidal center for z-axis
cx  = 0;     % elliposidal center for x-axis
alpha2 = 10;
alpha1 = 100;
alpha0 = 100;
alphaz = 1;

% [ax, ay, az] = ellipsoid(0, 0, cz, px, py, pz); figure;
% h = surf(ax,ay,az);
% set(h, 'FaceAlpha', 0.25);
% shading interp;
% axis equal;

%% Barrier Conditions
if use_QP_u1 == 0 && use_QP_txy == 0
    fprintf('\n ------------ NOT USING QP SOLVER AT ALL -------------------- \n\n');
elseif use_QP_u1 == 1 && use_QP_txy == 0
    fprintf('\n ------------ USING QP SOLVER FOR U1 ------------------------ \n\n');
elseif use_QP_txy == 1 && use_QP_u1 == 0
    fprintf('\n ------------ USING QP SOLVER FOR Tx & Ty ------------------- \n\n');
elseif use_QP_txy == 1 && use_QP_u1 == 1
    fprintf('\n ------------ USING QP SOLVER FOR BOTH ---------------------- \n\n');
end

%% Drone Flight

fprintf('Simulating drone flight ... ');
inner_loop = 10;
totTime = 0;

for i=1:length(x_ref)
%     i
    tStart = tic;
    %% Hover (altitude) controller
    [u1, zdd_cmd] = zController(z_ref(i), zd_ref(i), zdd_ref(i), droneState, kp_xyz, kd_xyz, m);
    
    %% Position (XY) controller
    x_info = [x_ref(i) xd_ref(i) xdd_ref(i)];
    y_info = [y_ref(i) yd_ref(i) ydd_ref(i)];
    [xdd_cmd, ydd_cmd] = xyController(x_info, y_info, droneState, kp_xyz, kd_xyz);
    
    % Do not use this control scheme if QP applied only for tx and ty
    if use_QP_u1 == 1 || use_QP_txy == 0
        u1 = m*sqrt(xdd_cmd^2 + ydd_cmd^2 + (zdd_cmd-g)^2);
    end
    u1_hist(i) = u1;
    
    %% Barrier certificate on z-component
    if use_QP_u1 == 1
        H = 1;
        f = -u1;
        
        x  = getX(droneState);      y   = getY(droneState);     z   = getZ(droneState);
        dx = getXdot(droneState);   dy  = getYdot(droneState);  dz  = getZdot(droneState);
        dx_hist(i) = dx;
        dy_hist(i) = dy;
        [R13, R23, R33, ~] = getRotCol3(droneState);
        
        %%         % Single barrier on Z-VELOCITY
        h = 1 - (dz/vz)^2 ;
        A = -2*dz*R33/(vz^2*m);
        b = alphaz*h - 2*dz*g/vz^2;
        % ______________________________________________________________
        
        %%         % Single barrier on Z-POSITION
%         h = 1 - ((z-cz)/pz)^2;
%         dh = -2*(z-cz)*dz/pz^2;
%         
%         A = -2*(z-cz)*R33/(m*pz^2) ;
%         b = alpha0*h + alpha1*dh - 2*(z-cz)*g/pz^2 - 2*dz^2/pz^2;
        % ______________________________________________________________
        
        %%         % Single barrier on Z-POSITION & Z-VELOCITY
        %         h = 1 - ((z-cz)/pz)^2 - (dz/vz)^2;
        %         A = -2*dz*R33/(m*vz^2);
        %         b = alpha*h - 2*dz*g/vz^2 - 2*dz*(z-cz)/pz^2;
        % ______________________________________________________________
        
        %% QP-solver on u1
        uBound = 2*g;
        lBound = 0;
        
        options = optimset('display', 'off');
        u1_QP = quadprog(H, f, A, b, [], [], lBound, uBound, [], options);
        
        if ~isempty(u1_QP)
            u1_QP_hist(i) = u1_QP;
            u1 = u1_QP;
        end
    end
    
    %% Attitude controller
    for j=1:inner_loop
        %% Get updated orientation angles and body velocity rates
        x  = getX(droneState);      y   = getY(droneState);     z   = getZ(droneState);
        dx = getXdot(droneState);   dy  = getYdot(droneState);  dz  = getZdot(droneState);
        [R13, R23, R33, R] = getRotCol3(droneState);
        R11 = R(1,1);   R12 = R(1,2);
        R21 = R(2,1);   R22 = R(2,2);
        phi = getPhi(droneState);   theta = getTheta(droneState);   psi = getPsi(droneState);
        p = getP(droneState);       q = getQ(droneState);           r = getR(droneState);
        
        %% Controllers
        % implementing roll-pitch and yaw controllers
        [p_cmd, q_cmd] = rollPitchController(...
            u1, xdd_cmd, ydd_cmd, phi, theta, psi, kp_roll_pitch, m);
        r_cmd = yawController(psi_ref(i), psi, kp_yaw);
        
        % implementing pqr (body-rate) controller
        [pd_cmd, qd_cmd, rd_cmd] = pqrController(p_cmd, q_cmd, r_cmd, p, q, r, kp_pqr);
        
        %% Set propeller speeds
        % Set angular velocities on each propeller
        I = [Ix Iy Iz];
        [w1, w2, w3, w4] = setAngularVelocities(u1, pd_cmd, qd_cmd, rd_cmd, droneState, I, kf, km, l);
        
        %% Compute forces and torque moments
        % Forces
        f1 = kf*w1^2;   f2 = kf*w2^2;   f3 = kf*w3^2;   f4 = kf*w4^2;
        Ftotal = f1 + f2 + f3 + f4;
        
        F_hist(i) = Ftotal;
        
        % Torques/Moments
        t1 =  km*w1^2;  t2 = -km*w2^2;  t3 =  km*w3^2;  t4 = -km*w4^2;
        
        tx = (f1 + f4 - f2 - f3)*l;
        ty = (f1 + f2 - f3 - f4)*l;
        tz = t1 + t2 + t3 + t4;
        tau = [tx ty tz];
        
        %% Implement barriers on Tau-x and Tau-y
        if use_QP_txy == 1
            H = eye(2);
            f = -[tx ty]';
            
            [R13dot, R23dot] = getRotCol3Deriv(R, p, q);
            dddx = -u1*R13dot/m;     % third derivative of x
            dddy = -u1*R23dot/m;     % third derivative of y
            
            ddx  = -u1*R13/m;       % second derivative of x
            ddy  = -u1*R23/m;       % second derivative of y
            
            % Derivative of rot matrix R_ZYX elements
            [R11dot, R12dot, R21dot, R22dot, R33dot] = getRotDerivative(droneState);
            W       = [ R21     -R11 ;  R22     -R12 ];
            Wdot    = [R21dot -R11dot; R22dot -R12dot];
            V       = pinv(W);
            Vdot    = get2x2MatrixDeriv(W, Wdot);
            % inertial terms for pdot and qdot
            dp_term1 = -q*r*(Iz-Iy) ;
            dq_term1 = -p*r*(Ix-Iz) ;
            
            % For simplified expressions
            Jx = -6*ddx*dddx/vx^2 ;
            Jy = -6*ddy*dddy/vy^2 ;
            u_by_m = u1/m ;
            
            L  = u_by_m*R33*V;
            K  = -u_by_m*R33dot*V*[p q]' - u_by_m*R33*Vdot*[p q]' - L*[dp_term1/Ix dq_term1/Iy]';
            Kx = K(1);
            Ky = K(2);
            
            Ixy = [1/Ix 0 ; 0 1/Iy] ;
            gamma = [2*dx/vx^2       2*dy/vy^2];
            
            %% Single barrier on X-VELOCITY & Y-VELOCITY
            h    = 1 - dx^2/vx^2 - dy^2/vy^2;
            dh   = -(2*ddx*dx)/vx^2 - (2*ddy*dy)/vy^2;
            ddh  = -(2*ddx^2)/vx^2 - (2*ddy^2)/vy^2 - (2*dddx*dx)/vx^2 - (2*dddy*dy)/vy^2;
            
            % Simplifying expressions
            beta = alpha2*ddh + alpha1*dh + alpha0*h ;
            
            A = -gamma* L * Ixy ;
            b = beta - gamma*K - Jx - Jy;
            
            % QP solver
            lBound = [-5.0 -5.0];
            uBound = [ 5.0  5.0];
            
            options = optimset('display', 'off');
            tau_QP = quadprog(H, f, A, b, [], [], lBound, uBound, [], options);
            tx_QP = tau_QP(1);
            ty_QP = tau_QP(2);
            
            
        end
        
        if use_QP_txy == 1
            tau = [tx_QP ty_QP tz];
            tau_hist_QP(i,:) = tau;
        elseif use_QP_txy == 0
            tau = [tx ty tz];
            tau_hist(i,:) = tau;
        end
        
        %% Update drone states
        droneState = updateDroneState(droneState+eps, I, tau, g, m, Ftotal, dt/inner_loop);
    end
    
    stateHistory(i,:) = droneState;
    timeElapsed = toc(tStart);
    totTime = totTime + timeElapsed;
end
avgTime = totTime/length(time);


%% Save states based on conditions
% EXAMPLE USAGE: save filename variable_name
if use_QP_u1 == 0 && use_QP_txy == 0
    x_noBarrier     = stateHistory(:,1);
    y_noBarrier     = stateHistory(:,2);
    z_noBarrier     = stateHistory(:,3);
    xdot_noBarrier  = stateHistory(:,7);
    ydot_noBarrier  = stateHistory(:,8);
    zdot_noBarrier  = stateHistory(:,9);
    state_noBarrier = [x_noBarrier y_noBarrier z_noBarrier xdot_noBarrier ydot_noBarrier zdot_noBarrier];
    save 'state_noBarrier' 'state_noBarrier'
else
    x_wBarrier      = stateHistory(:,1);
    y_wBarrier      = stateHistory(:,2);
    z_wBarrier      = stateHistory(:,3);
    xdot_wBarrier   = stateHistory(:,7);
    ydot_wBarrier   = stateHistory(:,8);
    zdot_wBarrier   = stateHistory(:,9);
    state_wBarrier  = [x_wBarrier y_wBarrier z_wBarrier xdot_wBarrier ydot_wBarrier zdot_wBarrier];
    save 'state_wBarrier' 'state_wBarrier'
end
save 'stateHistory' 'stateHistory'

fprintf('Done.              ');
toc
fprintf('Time per iteration = %4.3f milliseconds. \n\n', avgTime*1000);

%% Comparing Drone [x,y,z,yaw] against reference
tic
fprintf('Plotting drone behavior ... ');

if use_QP_u1 == 1 || use_QP_txy == 1
    row = 3;    col = 3;
elseif (use_QP_u1 == 0 || use_QP_txy == 0) && plot_flag == 1
    row = 2; col = 2;
else
    row = 1; col = 1;
end

%% Plotting based on conditions
if (plot_flag == 1)
    %% Comparing drone's trajectory with reference trajectory
    subplot(row, col,1);
    
    x_flight = stateHistory(:,1)';
    y_flight = stateHistory(:,2)';
    z_flight = stateHistory(:,3)';
    
    plot3(x_ref, y_ref, z_ref, 'r'); grid on; hold on;
    plot3(x_flight, y_flight, z_flight, 'b');
    
    title('Flight Path','Interpreter','Latex');
    xlabel('X [meters]','Interpreter','Latex');
    ylabel('Y [meters]','Interpreter','Latex');
    zlabel('Z [meters]','Interpreter','Latex');
    limit = 4;
    axis([  min(x_ref)-limit/4    max(x_ref)+limit/4 ...
        min(y_ref)-limit/4    max(y_ref)+limit/4 ...
        min(z_ref)-limit    max(z_ref)+limit   ]);
    
    
    % Plotting barrier ellipse
    %     [sx, sy, sz] = ellipsoid(0, 0, cz, px, py, pz);
    %     h = surf(sx,sy,sz);
    %     set(h, 'FaceAlpha', 0.25);
    %     shading interp;
    %     legend('Reference', 'Actual', 'Barrier Set');
    %         axis equal;
    
    view(30,30);
    
    %% Comparing behavior with and without barriers
    
    if use_QP_u1 || use_QP_txy
        load state_noBarrier.mat
        load state_wBarrier.mat
        x_noBarrier     = state_noBarrier(:,1);
        y_noBarrier     = state_noBarrier(:,2);
        z_noBarrier     = state_noBarrier(:,3);
        xdot_noBarrier  = state_noBarrier(:,4);
        ydot_noBarrier  = state_noBarrier(:,5);
        zdot_noBarrier  = state_noBarrier(:,6);
        
        color = 'black';
        
        subplot(row, col, 4);
        plot(time, x_noBarrier, 'r', time, x_wBarrier, 'b'); hold on;
        line([min(time),max(time)],[max(x_noBarrier), max(x_noBarrier)], 'Color',color,'LineStyle','--')
        line([min(time),max(time)],[min(x_noBarrier), min(x_noBarrier)], 'Color',color,'LineStyle','--')
        xlabel('Time','Interpreter','Latex');
        ylabel('x(t)','Interpreter','Latex');
        title('x(t)');
        legend('Without Barrier' , ['With Barrier = ' num2str(px) ], ['Ref limit = ' num2str(max(x_noBarrier)) ]);
        grid on;
        
        
        subplot(row, col, 5);
        plot(time, y_noBarrier, 'r', time, y_wBarrier, 'b'); hold on;
        line([min(time),max(time)],[max(y_noBarrier), max(y_noBarrier)], 'Color',color,'LineStyle','--')
        line([min(time),max(time)],[min(y_noBarrier), min(y_noBarrier)], 'Color',color,'LineStyle','--')
        xlabel('Time','Interpreter','Latex');
        ylabel('y(t)','Interpreter','Latex');
        title('y(t)');
        legend('Without Barrier' , ['With Barrier = ' num2str(py) ], ['Ref limit = ' num2str(max(y_noBarrier)) ]);
        grid on;
        
        
        subplot(row, col, 6);
        plot(time, z_noBarrier, 'r', time, z_wBarrier, 'b'); hold on;
        line([min(time),max(time)],[max(z_noBarrier), max(z_noBarrier)], 'Color',color,'LineStyle','--')
        line([min(time),max(time)],[min(z_noBarrier), min(z_noBarrier)], 'Color',color,'LineStyle','--')
        xlabel('Time','Interpreter','Latex');
        ylabel('z(t)','Interpreter','Latex');
        title('z(t)');
        legend('Without Barrier' , ['With Barrier = ' num2str(pz) ], ['Ref limit = ' num2str(max(z_noBarrier)) ]);
        grid on;
        
        
        
        subplot(row, col, 7);
        plot(time, xdot_noBarrier, 'r', time, xdot_wBarrier, 'b'); hold on;
        line([min(time),max(time)],[max(xdot_noBarrier), max(xdot_noBarrier)], 'Color',color,'LineStyle','--')
        line([min(time),max(time)],[min(xdot_noBarrier), min(xdot_noBarrier)], 'Color',color,'LineStyle','--')
        xlabel('Time','Interpreter','Latex');
        ylabel('$\dot{x}(t)$','Interpreter','Latex');
        title('xdot(t)');
        legend('Without Barrier' , ['With Barrier = ' num2str(vx) ], ['Ref limit = ' num2str(max(xdot_noBarrier)) ]);
        grid on;
        
        
        subplot(row, col, 8);
        plot(time, ydot_noBarrier, 'r', time, ydot_wBarrier, 'b'); hold on;
        line([min(time),max(time)],[max(ydot_noBarrier), max(ydot_noBarrier)], 'Color',color,'LineStyle','--')
        line([min(time),max(time)],[min(ydot_noBarrier), min(ydot_noBarrier)], 'Color',color,'LineStyle','--')
        xlabel('Time','Interpreter','Latex');
        ylabel('$\dot{y}(t)$','Interpreter','Latex');
        title('ydot(t)');
        legend('Without Barrier' , ['With Barrier = ' num2str(vy) ], ['Ref limit = ' num2str(max(ydot_noBarrier)) ]);
        grid on;
        
        
        subplot(row, col, 9);
        plot(time, zdot_noBarrier, 'r', time, zdot_wBarrier, 'b'); hold on;
        line([min(time),max(time)],[max(zdot_noBarrier), max(zdot_noBarrier)], 'Color',color,'LineStyle','--')
        line([min(time),max(time)],[min(zdot_noBarrier), min(zdot_noBarrier)], 'Color',color,'LineStyle','--')
        xlabel('Time','Interpreter','Latex');
        ylabel('$\dot{z}(t)$','Interpreter','Latex');
        title('zdot(t)');
        legend('Without Barrier' , ['With Barrier = ' num2str(vz) ], ['Ref limit = ' num2str(max(zdot_noBarrier)) ]);
        grid on;
    end
    
    if use_QP_u1 == 0 && use_QP_txy == 0
        %% Comparing drone's heading with reference heading
        subplot(row, col,2);
        
        u = cos(psi_ref);
        v = sin(psi_ref);
        w = zeros(1,length(psi_ref));
        
        drone_u = cos(stateHistory(:,6))';
        drone_v = sin(stateHistory(:,6))';
        drone_w = zeros(1, length(psi_ref));
        
        step = 10;
        quiver3(x_ref(1:step:end), y_ref(1:step:end), z_ref(1:step:end), ...
            u(1:step:end), v(1:step:end), w(1:step:end), 0.5, 'r');
        grid on; hold on;
        quiver3(x_flight(1:step:end), y_flight(1:step:end), z_flight(1:step:end), ...
            drone_u(1:step:end), drone_v(1:step:end), drone_w(1:step:end), 0.5, 'g');
        
        title('Drone heading','Interpreter','Latex');
        xlabel('X [meters]','Interpreter','Latex');
        ylabel('Y [meters]','Interpreter','Latex');
        zlabel('Z [meters]','Interpreter','Latex');
        legend('Reference', 'Actual');
        limit = 0.5;
        axis([  min(x_ref)-limit    max(x_ref)+limit ...
            min(y_ref)-limit    max(y_ref)+limit ...
            min(z_ref)-limit    max(z_ref)+limit   ]);
        
        view(30,30)
        
        %% Computing error in drone and reference trajectory
        subplot(row, col,3);
        
        err_x = (x_ref - x_flight).^2;
        err_y = (y_ref - y_flight).^2;
        err_z = (z_ref - z_flight).^2;
        
        err_pos = sqrt(err_x + err_y + err_z);
        plot(time, err_pos, 'b'); grid on;
        title('Error norm in drone flight','Interpreter','Latex');
        xlabel('Time (seconds)','Interpreter','Latex');
        ylabel('Error (meters)','Interpreter','Latex');
        limit = 0.1;
        axis([  min(time)-limit    max(time)+limit ...
            min(err_pos)-limit    max(err_pos)+limit ]);
        
        %% Comparing drone yaw and reference yaw against time
        subplot(row, col,4)
        
        drone_psi = stateHistory(:,6)';
        plot(time, psi_ref, 'r');
        grid on; hold on;
        plot(time, drone_psi, 'b');
        title('Yaw behavior','Interpreter','Latex');
        xlabel('Time (seconds)','Interpreter','Latex');
        ylabel('$\psi$ (radians)','Interpreter','Latex');
        legend('Reference', 'Actual');
    end
end
fprintf('Done.              ');
toc

