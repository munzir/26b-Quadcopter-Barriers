clc; clear; close all;

%% Drone characteristics
kf = 1.0;   % constant relating force and rotor speed
km = 1.0;   % constant relating torque and rotor speed
m  = 1.0;   % mass of quad in kg
g  = 9.81;  % gravity
L  = 0.25;   % rotor-to-rotor distance
l  = L/(2*sqrt(2));
Ix = 0.1;   % inertia along x-axis
Iy = 0.1;   % inertia along y-axis
Iz = 0.2;   % inertia along z-axis

% x,  y,  z,  phi, theta, psi
states = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ...
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
%vx, vy, vz,    p,    q,   r
omega  = [0.0, 0.0, 0.0, 0.0]; % rotor speeds

%% Reference Trajectory (Flight plan)
total_time = 20; % 20 seconds
dt = 0.01;
time = linspace(0, total_time, floor(total_time/dt) );

omega_x = 0.8; omega_y = 0.4; omega_z = 0.4;
x_arg   = omega_x*time;
x_ref   = sin(x_arg);
xd_ref  = omega_x*cos(x_arg);
xdd_ref = -omega_x*omega_x*sin(x_arg);

y_arg   = omega_y*time;
y_ref   = cos(y_arg);
yd_ref  = -omega_y*sin(y_arg);
ydd_ref = -omega_y*omega_y*cos(y_arg);

z_arg   = omega_z*time;
z_ref   = cos(z_arg);
zd_ref  = -omega_z*sin(z_arg);
zdd_ref = -omega_z*omega_z*cos(z_arg);

psi_ref = atan2(yd_ref, xd_ref);
refTrajectory = [x_ref' y_ref' z_ref' xd_ref' yd_ref' zd_ref' xdd_ref' ydd_ref' zdd_ref' psi_ref'];

save 'refTrajectory' 'refTrajectory'

%% PID parameters for each controller
% x, y, z kp and kd values
kp_xyz = [4.0, 4.0, 1.0];   % x, y, ,z kp values
kd_xyz = [2.0, 2.0, 1.0];   % x, y, z, kd values (2nd order)

kp_roll_pitch = [8.0, 8.0]; % roll, pitch kp values (1st order)
kp_yaw = 8.0;

kp_pqr = [20.0 20.0 20.0];  % p, q, r kp values (1st order)

droneState = [ x_ref(1)   y_ref(1)     z_ref(1) ...
    0.0        0.0    psi_ref(1) ...
    xd_ref(1)  yd_ref(1)    zd_ref(1) ...
    0.0        0.0       0.0];


%% Drone Flight
inner_loop = 10;
for i=1:length(x_ref)
    %% Hover (altitude) controller
    [~, zdd_cmd] = zController(z_ref(i), zd_ref(i), zdd_ref(i), droneState, kp_xyz, kd_xyz, m);
    
    %% Position (XY) controller
    x_info = [x_ref(i) xd_ref(i) xdd_ref(i)];
    y_info = [y_ref(i) yd_ref(i) ydd_ref(i)];
    [xdd_cmd, ydd_cmd] = xyController(x_info, y_info, droneState, kp_xyz, kd_xyz);
    
    u1 = m*sqrt(xdd_cmd^2 + ydd_cmd^2 + (zdd_cmd-g)^2);
    
    %% Attitude controller
    for j=1:inner_loop
        %% Get updated orientation angles and body velocity rates
        phi = getPhi(droneState);
        theta = getTheta(droneState);
        psi = getPsi(droneState);
        p = getP(droneState);
        q = getQ(droneState);
        r = getR(droneState);
        
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
        
        %% Update drone state
        % Forces
        f1 = kf*w1^2;   f2 = kf*w2^2;   f3 = kf*w3^2;   f4 = kf*w4^2;
        Ftotal = f1 + f2 + f3 + f4;
        
        F_hist(i) = Ftotal;
        u1_hist(i) = u1;
        
        % Torques/Moments
        t1 =  km*w1^2;  t2 = -km*w2^2;  t3 =  km*w3^2;  t4 = -km*w4^2;
        
        tx = (f1 + f4 - f2 - f3)*l;
        ty = (f1 + f2 - f3 - f4)*l;
        tz = t1 + t2 + t3 + t4;
        tau = [tx ty tz];
        
        droneState = updateDroneState(droneState, I, tau, g, m, Ftotal, dt/inner_loop);
    end
    stateHistory(i,:) = droneState;
end
disp('Drone simulation completed');

save 'stateHistory' 'stateHistory'

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Comparing Drone [x,y,z,yaw] against reference
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
plot_flag = 1;
if (plot_flag)
    
    %% Comparing drone's trajectory with reference trajectory
    subplot(2,2,1);
    
    x_flight = stateHistory(:,1)';
    y_flight = stateHistory(:,2)';
    z_flight = stateHistory(:,3)';
    
    
    plot3(x_ref, y_ref, z_ref, 'r'); grid on; hold on;
    plot3(x_flight, y_flight, z_flight, 'b');
    
    title('Flight Path','Interpreter','Latex');
    xlabel('X [meters]','Interpreter','Latex');
    ylabel('Y [meters]','Interpreter','Latex');
    zlabel('Z [meters]','Interpreter','Latex');
    legend('Reference', 'Actual');
    limit = 0.5;
    axis([  min(x_ref)-limit    max(x_ref)+limit ...
        min(y_ref)-limit    max(y_ref)+limit ...
        min(z_ref)-limit    max(z_ref)+limit   ]);
    
    view(30,30)
    
    %% Comparing drone's heading with reference heading
    subplot(2,2,2);
    
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
    subplot(2,2,3);
    
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
    subplot(2,2,4)
    
    drone_psi = stateHistory(:,6)';
    plot(time, psi_ref, 'r');
    grid on; hold on;
    plot(time, drone_psi, 'b');
    title('Yaw behavior','Interpreter','Latex');
    xlabel('Time (seconds)','Interpreter','Latex');
    ylabel('$\psi$ (radians)','Interpreter','Latex');
    legend('Reference', 'Actual');
    
end

